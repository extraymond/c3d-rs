use anyhow::Result;
use std::collections::HashMap;

use std::cell::RefCell;
use std::fs::File;
use std::io;
use std::io::prelude::*;
use std::io::Cursor;
use std::io::SeekFrom;
use std::mem;
use std::rc::Rc;
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
    #[error("missing header/parameter")]
    MissingField,
}

pub struct C3dAdapter<T: Read + Seek> {
    pub header: Option<HeaderBlock>,
    pub parameter: Option<ParameterBlock>,
    handle: Rc<RefCell<T>>,
}

impl<T: Read + Seek> C3dAdapter<T> {
    pub fn new(mut file: T) -> Result<Self, ParserError> {
        file.seek(SeekFrom::Start(0))?;
        let handle = Rc::new(RefCell::new(file));

        Ok(C3dAdapter {
            header: None,
            parameter: None,
            handle,
        })
    }

    pub fn construct(mut self) -> Result<Self, ParserError> {
        let header = HeaderBlock::from_reader(&mut *self.handle.borrow_mut());
        let parameter = ParameterBlock::from_reader(&mut *self.handle.borrow_mut());

        // 0x50 if header is of correct format.
        if header.magic_word != 0x50 {
            return Err(ParserError::UnmatchMagic);
        }

        // 0x50 + 4 if parameter is of correct format.
        if parameter.header.magic_word != 0x50 + 4 {
            dbg!(parameter.header.magic_word);
            return Err(ParserError::UnmatchMagic);
        }

        self.header.replace(header);
        self.parameter.replace(parameter);

        Ok(self)
    }
}

pub struct C3dReader<'a, R: Read + Seek> {
    header: &'a HeaderBlock,
    parameter: &'a ParameterBlock,
    handle: std::cell::RefMut<'a, R>,
    points_buffer: Vec<u8>,
    analog_buffer: Vec<u8>,
    frame_idx: u16,
    analog_unsigned: bool,
    analog_offset: Option<Vec<f32>>,
    analog_scale: Option<Vec<f32>>,
    analog_gen_scale: Option<f32>,
}

impl<'a, R: Read + Seek> C3dReader<'a, R> {
    pub fn new(
        header: &'a HeaderBlock,
        parameter: &'a ParameterBlock,
        mut handle: std::cell::RefMut<'a, R>,
    ) -> Result<Self, ParserError> {
        (*handle).seek(SeekFrom::Start((header.data_start as u64 - 1) * 512))?;
        let points_buffer: Vec<u8> = vec![];
        let analog_buffer: Vec<u8> = vec![];

        let analog_unsigned = parameter
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

        let analog_offset = parameter.get("ANALOG:OFFSET").map(|v| {
            v.parameter_data
                .values
                .iter()
                .filter_map(|arr| arr.as_i16())
                .map(|a| *a as f32)
                .collect::<Vec<f32>>()
        });

        let analog_scale = parameter.get("ANALOG:SCALE").map(|v| {
            v.parameter_data
                .values
                .iter()
                .filter_map(|arr| arr.as_f32())
                .map(|a| *a)
                .collect::<Vec<f32>>()
        });

        let analog_gen_scale = parameter
            .get("ANALOG:GEN_SCALE")
            .map(|v| {
                v.parameter_data
                    .values
                    .iter()
                    .filter_map(|arr| arr.as_f32())
                    .map(|a| *a)
                    .next()
            })
            .flatten();

        log::debug!("analog offsets: {:?}", analog_offset);
        log::debug!("scale factors: {:?}", analog_scale);
        log::debug!("genral scale factor: {:?}", analog_gen_scale);

        Ok(C3dReader {
            header,
            parameter,
            handle,
            points_buffer,
            analog_buffer,
            frame_idx: header.frame_first,
            analog_unsigned,
            analog_offset,
            analog_scale,
            analog_gen_scale,
        })
    }
}

impl<'a, R: Read + Seek> Iterator for C3dReader<'a, R> {
    type Item = (u16, PointData, Option<AnalogData>);

    fn next(&mut self) -> Option<Self::Item> {
        if self.frame_idx > self.header.frame_last {
            return None;
        }

        let point_scale = self.header.scale;
        let is_float = point_scale <= 0.0;

        let point_data_length = if is_float { 4 } else { 2 };

        let point_scale = if is_float { 1.0 } else { point_scale.abs() };

        let analog_data_length = if is_float { 4 } else { 2 };

        let points_n = 4 * self.header.point_counts;
        let analog_n = self.header.analog_counts;

        self.points_buffer
            .resize((points_n * point_data_length) as usize, 0_u8);

        if self.handle.read_exact(&mut self.points_buffer[..]).is_err() {
            return None;
        }

        let values = self
            .points_buffer
            .chunks_exact(4 * point_data_length as usize)
            .map(|arr| {
                let mut points_vec = [0_f32; 5];
                let raw_vec = arr
                    .chunks_exact(point_data_length as usize)
                    .filter_map(|arr| {
                        Some(if is_float {
                            let mut buf = [0_u8; 4];
                            arr.clone().read_exact(&mut buf).unwrap();
                            f32::from_le_bytes(buf)
                        } else {
                            let mut buf = [0_u8; 2];
                            arr.clone().read_exact(&mut buf).unwrap();
                            i16::from_le_bytes(buf) as f32 * point_scale
                        })
                    })
                    .collect::<Vec<f32>>();
                points_vec[..4].copy_from_slice(&raw_vec);
                points_vec
            })
            .map(|mut arr| {
                let fourth = &arr[3];

                if *fourth <= -0.01_f32 {
                    arr[3..5].iter_mut().for_each(|v| {
                        *v = -0.01_f32;
                    });
                } else {
                    let err = arr[3] as i16;
                    // the right eight bits are for error estimation.
                    arr[3] = (err & 0xff) as f32 * self.header.scale.abs();

                    // the left eight bits are for reporting total number of camera that obsered
                    arr[4] = (8..17)
                        .map(|idx| {
                            // camera mask to check wheter the bits are sets
                            let mask = idx << 8;

                            (err & mask) as f32
                        })
                        .sum();
                }
                arr
            })
            .collect::<Vec<_>>();

        let point_data = PointData { values };
        let analog_data = if analog_n > 0 {
            self.analog_buffer
                .resize((analog_n * analog_data_length) as usize, 0_u8);
            if self.handle.read_exact(&mut self.analog_buffer[..]).is_err() {
                return None;
            }

            let mut values: Vec<f32> = self
                .analog_buffer
                .chunks_exact(analog_data_length as usize)
                .map(|arr| {
                    if is_float {
                        let mut buf = [0_u8; 4];
                        (arr.clone()).read_exact(&mut buf).unwrap();
                        f32::from_le_bytes(buf)
                    } else {
                        let mut buf = [0_u8; 2];
                        if self.analog_unsigned {
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            u16::from_le_bytes(buf) as f32
                        } else {
                            (arr.clone()).read_exact(&mut buf).unwrap();
                            i16::from_le_bytes(buf) as f32
                        }
                    }
                })
                .collect();
            if let Some(offsets) = self.analog_offset.as_ref() {
                values.iter_mut().zip(offsets.iter()).for_each(|(v, off)| {
                    *v -= *off;
                });
            }

            if let Some(scales) = self.analog_scale.as_ref() {
                values.iter_mut().zip(scales.iter()).for_each(|(v, off)| {
                    *v *= *off;
                });
            }

            if let Some(gen_scale) = self.analog_gen_scale.as_ref() {
                values.iter_mut().for_each(|v| {
                    *v *= *gen_scale;
                });
            }

            Some(AnalogData { values })
        } else {
            None
        };

        let frame_idx = self.frame_idx;
        self.frame_idx += 1;

        Some((frame_idx, point_data, analog_data))
    }
}

impl<T: Read + Seek> C3dAdapter<T> {
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

    pub fn reader<'a>(&'a self) -> Result<C3dReader<'a, T>, ParserError> {
        if let Some(header) = self.header.as_ref() {
            if let Some(parameter) = self.parameter.as_ref() {
                let handle = self.handle.borrow_mut();
                return C3dReader::new(header, parameter, handle);
            }
        }
        Err(ParserError::MissingField)
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
                let param_data = ParamData {
                    values: datas,
                    locked,
                };

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
                        locked,
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
    magic_word: u8,
}

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
    locked: bool,
    params: HashMap<String, ParameterFormat>,
}

#[derive(Debug)]
pub struct ParamData {
    pub values: Vec<Box<dyn ParamValue>>,
    locked: bool,
}

#[derive(Debug)]
pub struct PointData {
    pub values: Vec<[f32; 5]>,
}

#[derive(Debug)]
pub struct AnalogData {
    pub values: Vec<f32>,
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

#[cfg(test)]
mod tests {
    use super::*;

    fn set_logger() {
        femme::with_level(log::LevelFilter::Debug);
    }

    #[test]
    fn test_parser() -> Result<()> {
        set_logger();

        let mut file = File::open("test_data/vicon_trial.c3d")?;
        let adapter = C3dAdapter::new(&mut file)?.construct()?;
        for (i, p, a) in adapter.reader()?.into_iter() {
            dbg!(i, p, a);
        }

        let mut file = File::open("test_data/motion_shadow.c3d")?;
        let adapter = C3dAdapter::new(&mut file)?.construct()?;
        for (i, p, a) in adapter.reader()?.into_iter() {
            dbg!(i, p, a);
        }

        Ok(())
    }
}
