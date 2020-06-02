# Readme

This is a c3d file format parser/reader written in rust. C3d is a biomechanics data format wildly used in the field of motion cpature.

The parser is written with the guidance of the [c3d.org](https://www.c3d.org/docs/C3D_User_Guide.pdf) specification and the python parser from [py-c3d](https://github.com/EmbodiedCognition/py-c3d).

**features**

1. Currently parsing only.
2. After reading the header/parameter block, because the adapter implements the iterator trait, you can read the (frame_index, points_data, analog_data) without needing to copy all the data section upfront.
3. Error estimation and camera observation information are correctly parsed according to the specification.
4. When consuming the reader, the analog data will be offset and scaled individulally/globally if the corresponding parameter is set.

**usage**

```rust

use c3d_rs::C3dAdapter;

let mut file = File::open("somefile")?;

let mut buf: Vec<u8> = vec![];
file.read_to_end(&mut buf)?;

let mut cursor = Cursor::new(buf)

/// adapter accepts impl Read + Seek.
let adapter = C3dAdapter::new(&mut file)?.construct()?;
let adapter = C3dAdapter::new(&mut buf[..])?.construct()?;


/// read labels into Vec<String> with whitespace stripped.
let point_labels: Vec<String> = adapter.get_points_labels().unwrap();
let adapter_labels: Vec<String> = adapter.get_adapter_labels().unwrap();

/// reading (frame, ponts, analog) from iterator
for (frame_idx, points_data, optional_analog_data) in adapter.reader()?.into_iter() {

}

/// working with vendor specific parameter.
let param = adapter.parameter.unwrap().get("GROUP:PARAMETER").unwrap();


```