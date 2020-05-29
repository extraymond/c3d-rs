use anyhow::Result;
use std::collections::HashMap;

use std::fs::File;
use std::io;
use std::io::prelude::*;
use std::io::Cursor;
use std::io::SeekFrom;
use std::mem;
use std::slice;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum ParserError {
    #[error("magic word not unmatched, might not be a c3d file")]
    UnmatchMagic,
    #[error("unable to parse paramter file")]
    ParseParameterError,
    #[error("io error")]
    IoError(#[from] io::Error),
}

#[derive(Default)]
pub struct C3dAdapter {
    pub header: Option<HeaderBlock>,
    pub parameter: Option<ParameterBlock>,
}

pub struct C3dReader<'a, 'b, R: Read + Seek> {
    header: &'a HeaderBlock,
    parameter: &'a ParameterBlock,
    handle: &'b mut R,
    points_buffer: Vec<u8>,
    analog_buffer: Vec<u8>,
    frame_idx: u16,
}

impl<'a, 'b, R: Read + Seek> C3dReader<'a, 'b, R> {
    pub fn new(
        header: &'a HeaderBlock,
        parameter: &'a ParameterBlock,
        handle: &'b mut R,
    ) -> Result<Self, ParserError> {
        handle.seek(SeekFrom::Start((header.data_start as u64 - 1) * 512))?;
        let points_buffer: Vec<u8> = vec![];
        let analog_buffer: Vec<u8> = vec![];

        Ok(C3dReader {
            header,
            parameter,
            handle,
            points_buffer,
            analog_buffer,
            frame_idx: header.frame_first,
        })
    }
}

impl<'a, 'b, R: Read + Seek> Iterator for C3dReader<'a, 'b, R> {
    type Item = (u16, PointData, Option<AnalogData>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.frame_idx > self.header.frame_last {
            return None;
        }

        let point_scale = self.header.scale;
        let is_float = point_scale < 0.0;

        let point_data_length = if is_float { 4 } else { 2 };
        let point_scale = if is_float { point_scale.abs() } else { 1.0 };

        let analog_data_length = if is_float { 4 } else { 2 };
        let is_unsigned = self
            .parameter
            .get("ANALOG:FORMAT")
            .map(|param| {
                param
                    .parameter_data
                    .values
                    .iter()
                    .filter_map(|v| v.as_char())
                    .filter(|c| !c.is_whitespace())
                    .collect::<String>()
            })
            .and(Some("UNSIGNED"))
            .is_some();

        let points_n = 4 * self.header.point_counts;
        let analog_n = self.header.analog_counts;

        self.points_buffer
            .resize((points_n * point_data_length) as usize, 0_u8);

        if self.handle.read_exact(&mut self.points_buffer[..]).is_err() {
            return None;
        }

        let values: Vec<Box<dyn DataValue>> = self
            .points_buffer
            .chunks_exact(point_data_length as usize)
            .filter_map(|arr| {
                Some(if is_float {
                    let mut buf = [0_u8; 4];
                    (arr.clone()).read_exact(&mut buf).unwrap();
                    Box::new(f32::from_le_bytes(buf) * point_scale) as Box<dyn DataValue>
                } else {
                    let mut buf = [0_u8; 2];
                    (arr.clone()).read_exact(&mut buf).unwrap();
                    Box::new(i16::from_le_bytes(buf) * point_scale as i16)
                })
            })
            .collect();

        let values = values
            .chunks_exact(4)
            .map(|v| v.iter().map(|v| v.clone_box()).collect())
            .collect();

        let point_data = PointData { values };
        let analog_data = if analog_n > 0 {
            self.analog_buffer
                .resize((analog_n * analog_data_length) as usize, 0_u8);
            if self.handle.read_exact(&mut self.analog_buffer[..]).is_err() {
                return None;
            }

            let values: Vec<Vec<Box<dyn AnalogValue>>> = self
                .analog_buffer
                .chunks_exact(analog_data_length as usize)
                .filter_map(|arr| {
                    Some(if is_float {
                        let mut buf = [0_u8; 4];
                        (arr.clone()).read_exact(&mut buf).unwrap();
                        Box::new(f32::from_le_bytes(buf)) as Box<dyn AnalogValue>
                    } else {
                        let mut buf = [0_u8; 2];
                        if is_unsigned {
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            Box::new(u16::from_le_bytes(buf)) as Box<dyn AnalogValue>
                        } else {
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            Box::new(i16::from_le_bytes(buf)) as Box<dyn AnalogValue>
                        }
                    })
                })
                .map(|v| vec![v])
                .collect();
            Some(AnalogData { values })
        } else {
            None
        };

        let frame_idx = self.frame_idx;
        self.frame_idx += 1;

        Some((frame_idx, point_data, analog_data))
    }
}

impl C3dAdapter {
    pub fn from_file(path: &str) -> Result<Self> {
        let mut file = File::open(path)?;
        let mut adapter = C3dAdapter::default();
        adapter.get_header(&mut file)?;
        adapter.get_parameter(&mut file)?;

        Ok(adapter)
    }

    pub fn get_header<T>(&mut self, r: &mut T) -> Result<()>
    where
        T: Seek + Read,
    {
        r.seek(SeekFrom::Start(0))?;
        let header = HeaderBlock::from_reader(r);
        self.header.replace(header);

        Ok(())
    }

    pub fn get_parameter<T>(&mut self, r: &mut T) -> Result<(), ParserError>
    where
        T: Seek + Read,
    {
        if let Some(header) = self.header.as_ref() {
            r.seek(SeekFrom::Start((header.parameter_start as u64 - 1) * 512))?;
            let parameter_block = ParameterBlock::from_reader(r);
            self.parameter.replace(parameter_block);
            Ok(())
        } else {
            Err(ParserError::ParseParameterError)
        }
    }

    pub fn get_point_lables(&self) -> Option<Vec<String>> {
        let mut rv = None;
        if let Some(parameter) = self.parameter.as_ref() {
            rv = parameter.get("POINT:LABELS").map(|param| {
                param
                    .parameter_data
                    .values
                    .chunks_exact(param.dimensions[0] as usize)
                    .map(|arr| {
                        arr.iter()
                            .filter_map(|v| v.as_char())
                            .filter(|c| !c.is_whitespace())
                            .collect::<String>()
                    })
                    .collect::<Vec<String>>()
            });
        }
        rv
    }

    pub fn get_analog_lables(&self) -> Option<Vec<String>> {
        let mut rv = None;
        if let Some(parameter) = self.parameter.as_ref() {
            if let Some(analog) = parameter.groups.get("ANALOG") {
                let mut keys = analog
                    .params
                    .keys()
                    .filter_map(|v| if v.contains("LABEL") { Some(v) } else { None })
                    .collect::<Vec<_>>();

                keys.sort();

                rv = Some(
                    keys.iter()
                        .filter_map(|v| {
                            parameter.get(&format!("ANALOG:{}", v)).map(|param| {
                                param
                                    .parameter_data
                                    .values
                                    .chunks_exact(param.dimensions[0] as usize)
                                    .map(|arr| {
                                        arr.iter()
                                            .filter_map(|c| c.as_char())
                                            .filter(|c| !c.is_whitespace())
                                            .collect::<String>()
                                    })
                                    .collect::<Vec<String>>()
                            })
                        })
                        .flatten()
                        .collect::<Vec<_>>(),
                );
            }
        }
        rv
    }

    pub fn reader<'a, 'b, R: Seek + Read>(
        &'a self,
        handle: &'b mut R,
    ) -> Result<C3dReader<'a, 'b, R>, ParserError> {
        C3dReader::new(
            self.header.as_ref().unwrap(),
            self.parameter.as_ref().unwrap(),
            handle,
        )
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct HeaderBlock {
    parameter_start: u8,
    magic_word: u8,
    point_counts: u16,
    analog_counts: u16,
    // index start from 1
    frame_first: u16,
    // index start from 1
    frame_last: u16,
    max_gap: u16,
    scale: f32,
    data_start: u16,
    analog_per_frame: u16,
    frame_rate: f32,
    reserved: [u8; 274],
    event_lables_long: u16,
    event_counts: u16,
    reserved_two: u16,
    event_times: [f32; 18],
    event_display_flags: [u8; 18],
    reserved_three: u16,
    event_labels: [u8; 72],
    reserved_four: [u8; 44],
}

trait FromReader {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self;
}

impl FromReader for HeaderBlock {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let mut header: HeaderBlock = unsafe { mem::zeroed() };
        // dbg!(&header);
        let header_size = mem::size_of::<HeaderBlock>();

        unsafe {
            // directly mutate the memory slice under Header.
            let header_slice =
                slice::from_raw_parts_mut(&mut header as *mut _ as *mut u8, header_size);

            r.read_exact(header_slice).unwrap();
        }

        header
    }
}

impl FromReader for ParameterBlockHeader {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let mut parameter: ParameterBlockHeader = unsafe { mem::zeroed() };

        unsafe {
            let parameter_slice = slice::from_raw_parts_mut(&mut parameter as *mut _ as *mut u8, 4);
            r.read_exact(parameter_slice).unwrap();
        }

        parameter
    }
}

impl FromReader for ParameterBlock {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let header = ParameterBlockHeader::from_reader(r);

        let mut u8_buffer = [0_u8];
        let mut i8_buffer = [0_u8];
        let mut i16_buffer = [0_u8; 2];
        let mut string_buffer: Vec<u8> = vec![];
        let mut parameter_buf: Vec<u8> = vec![];

        parameter_buf.resize(header.parameter_block_counts as usize * 512 - 4, 0);
        r.read_exact(&mut parameter_buf).unwrap();
        let mut parameter_block_cursor = Cursor::new(&parameter_buf[..]);

        let mut groups = HashMap::<u8, GroupFormat>::new();

        loop {
            parameter_block_cursor.read_exact(&mut i8_buffer).unwrap();
            let name_chars_size = i8::from_le_bytes(i8_buffer);

            let locked = name_chars_size < 0;
            let name_chars_size = name_chars_size.abs() as usize;

            parameter_block_cursor.read_exact(&mut i8_buffer).unwrap();
            let id: i8 = i8::from_le_bytes(i8_buffer);

            if id == 0 || name_chars_size == 0 {
                break;
            }

            string_buffer.resize(name_chars_size, 0_u8);
            parameter_block_cursor
                .read_exact(&mut string_buffer)
                .unwrap();

            let name = String::from_utf8(string_buffer.clone()).unwrap();

            parameter_block_cursor.read_exact(&mut i16_buffer).unwrap();
            let offset = i16::from_le_bytes(i16_buffer);

            let is_param = id > 0;
            if is_param {
                // length of each data element
                parameter_block_cursor.read_exact(&mut i8_buffer).unwrap();
                let data_length = i8::from_le_bytes(i8_buffer);

                // number of dimensions to read
                parameter_block_cursor.read_exact(&mut u8_buffer).unwrap();
                let num_dimensions = u8_buffer[0];

                let mut num_elements = 1_i32;
                let mut buf = [0_u8; 1];

                let mut dimensions = vec![];
                for _ in 0..num_dimensions {
                    parameter_block_cursor.read_exact(&mut buf).unwrap();
                    dimensions.push(buf[0]);
                    num_elements *= buf[0] as i32;
                }

                let total_data_length = num_elements as i16 * data_length.abs() as i16;

                let mut data_buffer = vec![0_u8; total_data_length as usize];
                parameter_block_cursor.read_exact(&mut data_buffer).unwrap();

                let datas: Vec<Box<dyn ParamValue>> = data_buffer
                    .chunks_exact(data_length.abs() as usize)
                    .filter_map(|arr| match data_length {
                        1 => Some(Box::new(arr[0] as u8) as Box<dyn ParamValue>),
                        2 => {
                            let mut buf = [0_u8; 2];
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            let val = i16::from_le_bytes(buf);
                            Some(Box::new(val) as Box<dyn ParamValue>)
                        }
                        4 => {
                            let mut buf = [0_u8; 4];
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            let val = f32::from_le_bytes(buf);
                            Some(Box::new(val))
                        }
                        -1 => Some(Box::new(arr[0] as char) as Box<dyn ParamValue>),
                        _ => None,
                    })
                    .collect();
                let param_data = ParamData { values: datas };

                parameter_block_cursor.read_exact(&mut u8_buffer).unwrap();
                let desc_chars_size = u8_buffer[0];
                string_buffer.resize(desc_chars_size as usize, 0);
                let desc = String::from_utf8(string_buffer.clone()).unwrap();

                let param = ParameterFormat {
                    id,
                    name_chars_size: name_chars_size as u8,
                    name: name.clone(),
                    offset,
                    data_length,
                    num_dimensions,
                    dimensions,
                    parameter_data: param_data,
                    desc_chars_size,
                    description: desc,
                };

                let group_id = id as u8;

                if let Some(group) = groups.get_mut(&group_id) {
                    group.params.insert(name, param);
                } else {
                    let mut group = GroupFormat::default();
                    group.params.insert(name, param);
                    groups.insert(group_id, group);
                }
            } else {
                let group_id = id.abs() as u8;
                parameter_block_cursor.read_exact(&mut u8_buffer).unwrap();
                let desc_chars_size = u8_buffer[0];
                string_buffer.resize(desc_chars_size as usize, 0);
                let desc = String::from_utf8(string_buffer.clone()).unwrap();

                if let Some(group) = groups.get_mut(&group_id) {
                    group.name = name;
                    group.description = desc;
                } else {
                    let new_group = GroupFormat {
                        name,
                        description: desc,
                        ..Default::default()
                    };
                    groups.insert(group_id, new_group);
                }
            }

            parameter_buf = parameter_buf.split_off(2 + name_chars_size as usize + offset as usize);
            parameter_block_cursor = Cursor::new(&parameter_buf[..]);
        }

        let groups: HashMap<String, GroupFormat> = groups
            .into_iter()
            .map(|(_, v)| (v.name.clone(), v))
            .collect();

        let parameter_block = ParameterBlock { header, groups };

        parameter_block
    }
}

#[derive(Debug)]
pub struct ParameterBlock {
    header: ParameterBlockHeader,
    groups: HashMap<String, GroupFormat>,
}

impl ParameterBlock {
    pub fn get(&self, key: &str) -> Option<&ParameterFormat> {
        let split_key: &'static str;

        if key.contains(".") {
            split_key = ".";
        } else if key.contains(":") {
            split_key = ":";
        } else {
            return None;
        }

        let mut iter = key.split(split_key);
        let group_key = iter.next().unwrap();
        let param_key = iter.next().unwrap();

        if let Some(group) = self.groups.get(group_key) {
            if let Some(param) = group.params.get(param_key) {
                return Some(&param);
            }
        }

        None
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone, Debug)]
struct ParameterBlockHeader {
    reserved_one: u8,
    reserved_two: u8,
    parameter_block_counts: u8,
    magic_number: u8,
}

// #[derive(Debug)]
// pub struct GroupParse {
//     name_chars_size: u8,
//     id: i8,
//     name: String,
//     offset: i16,
//     desc_chars_size: u8,
//     description: String,
//     params: HashMap<String, Parameter>,
// }

// pub struct GroupFormat {
//     // indicates "locked" if value is negative.
//     name_chars_size: i8,
//     id: i8,
//     name: String,
//     offset: i16,
//     desc_chars_size: u8,
//     description: String,
// }

#[derive(Debug)]
pub struct ParameterFormat {
    // indicates "locked" if value is negative.
    name_chars_size: u8,
    id: i8,
    name: String,
    offset: i16,
    data_length: i8,
    num_dimensions: u8,
    dimensions: Vec<u8>,
    parameter_data: ParamData,
    desc_chars_size: u8,
    description: String,
}

#[derive(Default, Debug)]
pub struct GroupFormat {
    name: String,
    description: String,
    params: HashMap<String, ParameterFormat>,
}

#[derive(Debug)]
pub struct ParamData {
    pub values: Vec<Box<dyn ParamValue>>,
}

#[derive(Debug)]
pub struct PointData {
    pub values: Vec<Vec<Box<dyn DataValue>>>,
}

#[derive(Debug)]
pub struct AnalogData {
    pub values: Vec<Vec<Box<dyn AnalogValue>>>,
}

pub trait ParamValue: std::fmt::Debug + ParamClone {
    fn as_char(&self) -> Option<&char> {
        None
    }
    fn as_f32(&self) -> Option<&f32> {
        None
    }
    fn as_i16(&self) -> Option<&i16> {
        None
    }
    fn as_u8(&self) -> Option<&u8> {
        None
    }
}

pub trait ParamClone {
    fn clone_box(&self) -> Box<dyn ParamValue>;
}

impl<T> ParamClone for T
where
    T: 'static + ParamValue + Clone,
{
    fn clone_box(&self) -> Box<dyn ParamValue> {
        Box::new(self.clone())
    }
}

impl ParamValue for char {
    fn as_char(&self) -> Option<&char> {
        Some(self)
    }
}
impl ParamValue for f32 {
    fn as_f32(&self) -> Option<&f32> {
        Some(self)
    }
}

impl ParamValue for i16 {
    fn as_i16(&self) -> Option<&i16> {
        Some(&self)
    }
}

impl ParamValue for u8 {
    fn as_u8(&self) -> Option<&u8> {
        Some(&self)
    }
}

pub trait DataClone {
    fn clone_box(&self) -> Box<dyn DataValue>;
}

impl<T> DataClone for T
where
    T: 'static + DataValue + Clone,
{
    fn clone_box(&self) -> Box<dyn DataValue> {
        Box::new(self.clone())
    }
}

pub trait DataValue: std::fmt::Debug + DataClone {
    fn as_f32(&self) -> Option<&f32> {
        None
    }
    fn as_i16(&self) -> Option<&i16> {
        None
    }
}

impl DataValue for f32 {
    fn as_f32(&self) -> Option<&f32> {
        Some(self)
    }
}
impl DataValue for i16 {
    fn as_i16(&self) -> Option<&i16> {
        Some(self)
    }
}

pub trait AnalogClone {
    fn clone_box(&self) -> Box<dyn AnalogValue>;
}

impl<T> AnalogClone for T
where
    T: 'static + AnalogValue + Clone,
{
    fn clone_box(&self) -> Box<dyn AnalogValue> {
        Box::new(self.clone())
    }
}

pub trait AnalogValue: std::fmt::Debug {
    fn as_f32(&self) -> Option<&f32> {
        None
    }
    fn as_i16(&self) -> Option<&i16> {
        None
    }
    fn as_u16(&self) -> Option<&u16> {
        None
    }
}

impl AnalogValue for f32 {
    fn as_f32(&self) -> Option<&f32> {
        Some(self)
    }
}
impl AnalogValue for u16 {
    fn as_u16(&self) -> Option<&u16> {
        Some(self)
    }
}
impl AnalogValue for i16 {
    fn as_i16(&self) -> Option<&i16> {
        Some(self)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_magic() -> Result<()> {
        let mut file = File::open("test_data/001.c3d")?;

        let mut adapter = C3dAdapter::default();
        adapter.get_header(&mut file)?;
        adapter.get_parameter(&mut file)?;

        for i in adapter
            .reader(&mut file)?
            .into_iter()
            .filter_map(|val| val.2)
        {
            i.values
                .iter()
                .zip(adapter.get_analog_lables().unwrap().iter())
                .filter(|(_, k)| k.contains("Tracker"))
                .for_each(|v| {
                    println!("{}: {:?}", v.1, &v.0[0]);
                })
        }

        Ok(())
    }
}
