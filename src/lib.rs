use std::collections::HashMap;
use std::fs::File;
use std::io;
use std::io::prelude::*;
use std::io::Cursor;
use std::io::SeekFrom;
use std::mem;
use std::slice;
use thiserror::Error;

use std::fmt;

#[derive(Error, Debug)]
pub enum ParserError {
    #[error("magic word not unmatched, might not be a c3d file")]
    UnmatchMagic,
}

pub struct C3d {
    pub header: Header,
    pub parameter: Vec<Parameter>,
    pub data: Vec<Data>,
}

impl C3d {
    pub fn read_bytes(bytes: &[u8]) -> Result<C3d, ParserError> {
        femme::with_level(log::LevelFilter::Info);
        let parameter_start = bytes[0];
        log::info!("parameter start: {}", parameter_start);

        let magic_word = bytes[1];

        if magic_word == 80 {
            return Err(ParserError::UnmatchMagic);
        }

        unimplemented!()
    }
}

#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct Header {
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

pub trait FromReader {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self;
}

impl FromReader for Header {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let mut header: Header = unsafe { mem::zeroed() };
        // dbg!(&header);
        let header_size = mem::size_of::<Header>();

        unsafe {
            // directly mutate the memory slice under Header.
            let header_slice =
                slice::from_raw_parts_mut(&mut header as *mut _ as *mut u8, header_size);

            r.read_exact(header_slice).unwrap();
        }

        header
    }
}

impl FromReader for ParameterHeader {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let mut parameter: ParameterHeader = unsafe { mem::zeroed() };

        unsafe {
            let parameter_slice = slice::from_raw_parts_mut(&mut parameter as *mut _ as *mut u8, 4);
            r.read_exact(parameter_slice).unwrap();
        }

        parameter
    }
}

impl FromReader for ParameterBlock {
    fn from_reader<R: Read + Seek>(r: &mut R) -> Self {
        let header = ParameterHeader::from_reader(r);

        let mut u8_buffer = [0_u8];
        let mut i8_buffer = [0_u8];
        let mut i16_buffer = [0_u8; 2];
        let mut string_buffer: Vec<u8> = vec![];
        let mut parameter_buf: Vec<u8> = vec![];

        parameter_buf.resize(header.parameter_block_counts as usize * 512 - 4, 0);
        r.read_exact(&mut parameter_buf).unwrap();
        let mut parameter_block_cursor = Cursor::new(&parameter_buf[..]);

        let mut groups = HashMap::<u8, Group>::new();

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

                let datas: Vec<ParameterData> = data_buffer
                    .chunks_exact(data_length.abs() as usize)
                    .map(|arr| ParameterData::from_array(data_length, arr).unwrap())
                    .collect();

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
                    parameter_data: datas,
                    desc_chars_size,
                    description: desc,
                };

                let group_id = id as u8;

                if let Some(group) = groups.get_mut(&group_id) {
                    group.params.insert(name, param);
                } else {
                    let mut group = Group::default();
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
                    let new_group = Group {
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

        let groups: HashMap<String, Group> = groups
            .into_iter()
            .map(|(_, v)| (v.name.clone(), v))
            .collect();

        let parameter_block = ParameterBlock { header, groups };

        parameter_block
    }
}

#[derive(Debug)]
pub struct ParameterBlock {
    header: ParameterHeader,
    groups: HashMap<String, Group>,
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
pub struct ParameterHeader {
    reserved_one: u8,
    reserved_two: u8,
    parameter_block_counts: u8,
    magic_number: u8,
}

// #[derive(Debug)]
pub struct GroupParse {
    name_chars_size: u8,
    id: i8,
    name: String,
    offset: i16,
    desc_chars_size: u8,
    description: String,
    params: HashMap<String, Parameter>,
}

pub struct GroupFormat {
    // indicates "locked" if value is negative.
    name_chars_size: i8,
    id: i8,
    name: String,
    offset: i16,
    desc_chars_size: u8,
    description: String,
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
    parameter_data: Vec<ParameterData>,
    desc_chars_size: u8,
    description: String,
}

#[derive(Default, Debug)]
pub struct Group {
    name: String,
    description: String,
    params: HashMap<String, ParameterFormat>,
}

#[derive(Default, Debug)]
pub struct Parameter {
    name: String,
    description: String,
}

#[derive(Debug, Clone)]
pub enum ParameterData {
    Character(char),
    Bytes(u8),
    Integer(i16),
    Float(f32),
}

impl ParameterData {
    pub fn from_array(data_length: i8, arr: &[u8]) -> Option<Self> {
        match data_length {
            -1 => {
                let content = arr[0] as char;
                Some(ParameterData::Character(content))
            }
            1 => Some(ParameterData::Bytes(arr[0])),
            2 => {
                let mut buf = [0_u8; 2];
                arr.clone().read_exact(&mut buf).unwrap();
                Some(ParameterData::Integer(i16::from_le_bytes(buf)))
            }
            4 => {
                let mut buf = [0_u8; 4];
                arr.clone().read_exact(&mut buf).unwrap();
                Some(ParameterData::Float(f32::from_le_bytes(buf)))
            }
            _ => None,
        }
    }
}

pub struct Data {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn test_magic() {
        let user = std::env::var("USER").unwrap();
        let mut file = File::open(format!("/home/{}/Downloads/takes/001.c3d", user)).unwrap();

        let header = Header::from_reader(&mut file);
        assert_eq!(header.magic_word, 80);

        let parameter = ParameterHeader::from_reader(&mut file);
        assert_eq!(parameter.magic_number, 84);

        file.seek(SeekFrom::Current(-4)).unwrap();
        let parameter_block = ParameterBlock::from_reader(&mut file);

        let param_datas = parameter_block.get("POINT:LABELS").unwrap();
        let lables = param_datas
            .parameter_data
            .chunks_exact(param_datas.dimensions[0] as usize)
            .map(|arr| {
                arr.iter()
                    .map(|v| {
                        if let ParameterData::Character(c) = v {
                            Some(c)
                        } else {
                            None
                        }
                    })
                    .map(|el| el.unwrap())
                    .collect::<String>()
            })
            .collect::<Vec<String>>();
        dbg!(lables);
    }
}
