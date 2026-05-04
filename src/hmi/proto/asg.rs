use std::{
    fmt::Display,
    hash::{Hash, Hasher},
};

use crate::hmi::{HmiError, hmi_handle::HmiResult, proto::wire::Message};

#[doc(hidden)]
pub(crate) mod __private {
    use super::*;
    pub trait Sealed {}
    macro_rules! sealer {
        ($($ty:ty),*) => {
            $(
                impl Sealed for $ty {}
            )*
        };
    }
    sealer! {
        bool,
        i8,
        i16,
        u16,
        i32,
        f32,
        f64,
        String,
        CartesianData,
        JointData,
        FrameData,
        PositionData,
        AlarmData,
        TimeData,
        ProgramStatus
    }
}

pub trait HmiWireable: Sized + __private::Sealed {
    const PACKED_SIZE: usize;
    const SYS_VAR_SIZE: usize;
    fn pack(&self, dst: &mut [u8]) -> usize;
    fn pack_sysvar(&self, dst: &mut [u8]) -> usize {
        self.pack(dst)
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError>;
    fn unpack_sysvar(src: &[u8]) -> Result<(Self, usize), HmiError> {
        Self::unpack(src)
    }
    fn partial_pack<const N: usize>(
        &self,
        start_offset: usize,
        view_size: usize,
        dst: &mut [u8],
    ) -> Result<usize, HmiError> {
        let mut filler = [0u8; N];
        let packed_size = self.pack(&mut filler);
        if view_size > packed_size || start_offset + view_size > N {
            log::error!(
                "Malformed response: cannot partial pack with view_size {} from packed size {} and start_offset {}",
                view_size,
                packed_size,
                start_offset
            );
            return Err(HmiError::MalformedResponse);
        }
        dst[..view_size].copy_from_slice(&filler[start_offset..start_offset + view_size]);
        Ok(view_size)
    }
    fn partial_pack_sysvar<const N: usize>(
        &self,
        start_offset: usize,
        view_size: usize,
        dst: &mut [u8],
    ) -> Result<usize, HmiError> {
        self.partial_pack::<N>(start_offset, view_size, dst)
    }
    fn partial_unpack<const N: usize>(
        src: &[u8],
        start_offset: usize,
        view_size: usize,
    ) -> Result<(Self, usize), HmiError> {
        let mut filler = [0u8; N];
        if view_size > src.len() || start_offset + view_size > Self::PACKED_SIZE {
            log::error!(
                "Malformed response: cannot partial unpack with view_size {} from src of size {} and start_offset {}",
                view_size,
                src.len(),
                start_offset
            );
            return Err(HmiError::MalformedResponse);
        }
        filler[start_offset..start_offset + view_size].copy_from_slice(&src[..view_size]);
        let (val, _) = Self::unpack(&filler)?;
        Ok((val, view_size))
    }
    fn partial_unpack_sysvar<const N: usize>(
        src: &[u8],
        start_offset: usize,
        view_size: usize,
    ) -> Result<(Self, usize), HmiError> {
        Self::partial_unpack::<N>(src, start_offset, view_size)
    }
}

impl HmiWireable for f32 {
    const PACKED_SIZE: usize = 4;
    const SYS_VAR_SIZE: usize = 4;
    fn pack(&self, dst: &mut [u8]) -> usize {
        let bytes = self.to_le_bytes();
        dst[..4].copy_from_slice(&bytes);
        4
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.len() < 4 {
            log::error!(
                "Malformed response: expected at least 4 bytes for f32, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&src[..4]);
        Ok((f32::from_le_bytes(bytes), 4))
    }
}

impl HmiWireable for i32 {
    const PACKED_SIZE: usize = 4;
    const SYS_VAR_SIZE: usize = 4;
    fn pack(&self, dst: &mut [u8]) -> usize {
        let bytes = self.to_le_bytes();
        dst[..4].copy_from_slice(&bytes);
        4
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.len() < 4 {
            log::error!(
                "Malformed response: expected at least 4 bytes for i32, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        let mut bytes = [0u8; 4];
        bytes.copy_from_slice(&src[..4]);
        Ok((i32::from_le_bytes(bytes), 4))
    }
}

impl HmiWireable for i16 {
    const PACKED_SIZE: usize = 2;
    const SYS_VAR_SIZE: usize = i32::SYS_VAR_SIZE;
    fn pack(&self, dst: &mut [u8]) -> usize {
        let bytes = self.to_le_bytes();
        dst[..2].copy_from_slice(&bytes);
        2
    }
    fn pack_sysvar(&self, dst: &mut [u8]) -> usize {
        (*self as i32).pack_sysvar(dst)
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.len() < 2 {
            log::error!(
                "Malformed response: expected at least 2 bytes for i16, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&src[..2]);
        Ok((i16::from_le_bytes(bytes), 2))
    }
    fn unpack_sysvar(src: &[u8]) -> Result<(Self, usize), HmiError> {
        let (val, consumed) = i32::unpack_sysvar(src)?;
        Ok((val as i16, consumed))
    }
}

impl HmiWireable for u16 {
    const PACKED_SIZE: usize = 2;
    const SYS_VAR_SIZE: usize = i32::SYS_VAR_SIZE;
    fn pack(&self, dst: &mut [u8]) -> usize {
        let bytes = self.to_le_bytes();
        dst[..2].copy_from_slice(&bytes);
        2
    }
    fn pack_sysvar(&self, dst: &mut [u8]) -> usize {
        (*self as i32).pack_sysvar(dst)
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.len() < 2 {
            log::error!(
                "Malformed response: expected at least 2 bytes for u16, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        let mut bytes = [0u8; 2];
        bytes.copy_from_slice(&src[..2]);
        Ok((u16::from_le_bytes(bytes), 2))
    }
    fn unpack_sysvar(src: &[u8]) -> Result<(Self, usize), HmiError> {
        let (val, consumed) = i32::unpack_sysvar(src)?;
        Ok((val as u16, consumed))
    }
}

impl HmiWireable for i8 {
    const PACKED_SIZE: usize = 1;
    const SYS_VAR_SIZE: usize = i32::SYS_VAR_SIZE;
    fn pack(&self, dst: &mut [u8]) -> usize {
        dst[0] = *self as u8;
        1
    }
    fn pack_sysvar(&self, dst: &mut [u8]) -> usize {
        (*self as i32).pack_sysvar(dst)
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.is_empty() {
            log::error!(
                "Malformed response: expected at least 1 byte for i8, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        Ok((src[0] as i8, 1))
    }
    fn unpack_sysvar(src: &[u8]) -> Result<(Self, usize), HmiError> {
        let (val, consumed) = i32::unpack_sysvar(src)?;
        Ok((val as i8, consumed))
    }
}

impl HmiWireable for bool {
    const PACKED_SIZE: usize = 1;
    const SYS_VAR_SIZE: usize = i32::SYS_VAR_SIZE;
    fn pack(&self, dst: &mut [u8]) -> usize {
        dst[0] = if *self { 1 } else { 0 };
        1
    }
    fn pack_sysvar(&self, dst: &mut [u8]) -> usize {
        (*self as i16).pack_sysvar(dst)
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.is_empty() {
            log::error!(
                "Malformed response: expected at least 1 byte for bool, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        Ok((src[0] != 0, 1))
    }
    fn unpack_sysvar(src: &[u8]) -> Result<(Self, usize), HmiError> {
        let (val, consumed) = i32::unpack_sysvar(src)?;
        Ok((val != 0, consumed))
    }
}

impl HmiWireable for String {
    const PACKED_SIZE: usize = 80;
    const SYS_VAR_SIZE: usize = 80;
    #[allow(clippy::needless_range_loop)]
    fn pack(&self, dst: &mut [u8]) -> usize {
        let bytes = self.as_bytes();
        let len = bytes.len().min(80);
        dst[..len].copy_from_slice(&bytes[..len]);
        for i in len..80 {
            dst[i] = 0;
        }
        80
    }
    fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
        if src.len() < 80 {
            log::error!(
                "Malformed response: expected at least 80 bytes for String, got {} bytes",
                src.len()
            );
            return Err(HmiError::MalformedResponse);
        }
        let str_bytes = &src[..80];
        let str_end = str_bytes.iter().position(|&b| b == 0).unwrap_or(80);
        let s = String::from_utf8_lossy(&str_bytes[..str_end]).to_string();
        Ok((s, 80))
    }
}

pub(crate) fn bytes_to_i16(bytes: &[u8]) -> Vec<i16> {
    bytes
        .chunks_exact(2)
        .map(|c| i16::from_le_bytes([c[0], c[1]]))
        .collect()
}

pub mod position_struct {
    use int_enum::IntEnum;

    use crate::hmi::{HmiError, asg::HmiWireable};

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(u16)]
    pub enum FlipState {
        Unflipped = 0,
        Flipped = 1,
    }

    impl std::fmt::Display for FlipState {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                FlipState::Unflipped => "Unflipped",
                FlipState::Flipped => "Flipped",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(u16)]
    pub enum LeftRight {
        Right = 0,
        Left = 1,
    }

    impl std::fmt::Display for LeftRight {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                LeftRight::Right => "Right",
                LeftRight::Left => "Left",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(u16)]
    pub enum UpDown {
        Down = 0,
        Up = 1,
    }

    impl std::fmt::Display for UpDown {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                UpDown::Down => "Down",
                UpDown::Up => "Up",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(u16)]
    pub enum FrontBack {
        Back = 0,
        Front = 1,
    }

    impl std::fmt::Display for FrontBack {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                FrontBack::Back => "Back",
                FrontBack::Front => "Front",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, set_all, str))]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    pub struct CartesianData {
        pub x: f32,
        pub y: f32,
        pub z: f32,
        pub w: f32,
        pub p: f32,
        pub r: f32,
        pub e1: f32,
        pub e2: f32,
        pub e3: f32,
        pub flip: FlipState,
        pub lr: LeftRight,
        pub ud: UpDown,
        pub fb: FrontBack,
        pub turn4: i16,
        pub turn5: i16,
        pub turn6: i16,
        pub is_valid: bool,
    }

    impl std::fmt::Display for CartesianData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "CartesianData {{ x: {}, y: {}, z: {}, w: {}, p: {}, r: {}, e1: {}, e2: {}, e3: {}, flip: {}, lr: {}, ud: {}, fb: {}, turn4: {}, turn5: {}, turn6: {}, is_valid: {} }}",
                self.x,
                self.y,
                self.z,
                self.w,
                self.p,
                self.r,
                self.e1,
                self.e2,
                self.e3,
                self.flip,
                self.lr,
                self.ud,
                self.fb,
                self.turn4,
                self.turn5,
                self.turn6,
                self.is_valid
            )
        }
    }

    impl HmiWireable for CartesianData {
        const PACKED_SIZE: usize = 52;
        const SYS_VAR_SIZE: usize = 52;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.x.pack(&mut dst[offset..]);
            offset += self.y.pack(&mut dst[offset..]);
            offset += self.z.pack(&mut dst[offset..]);
            offset += self.w.pack(&mut dst[offset..]);
            offset += self.p.pack(&mut dst[offset..]);
            offset += self.r.pack(&mut dst[offset..]);
            offset += self.e1.pack(&mut dst[offset..]);
            offset += self.e2.pack(&mut dst[offset..]);
            offset += self.e3.pack(&mut dst[offset..]);
            offset += (self.flip as u16).pack(&mut dst[offset..]);
            offset += (self.lr as u16).pack(&mut dst[offset..]);
            offset += (self.ud as u16).pack(&mut dst[offset..]);
            offset += (self.fb as u16).pack(&mut dst[offset..]);
            offset += self.turn4.pack(&mut dst[offset..]);
            offset += self.turn5.pack(&mut dst[offset..]);
            offset += self.turn6.pack(&mut dst[offset..]);
            offset += 1;
            offset += self.is_valid.pack(&mut dst[offset..]);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
            let mut offset = 0;
            let (x, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (y, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (z, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (w, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (p, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (r, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (e1, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (e2, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (e3, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (flip_u16, sz) = u16::unpack(&src[offset..])?;
            offset += sz;
            let flip = FlipState::try_from(flip_u16).map_err(|_| HmiError::MalformedResponse)?;
            let (lr_u16, sz) = u16::unpack(&src[offset..])?;
            offset += sz;
            let lr = LeftRight::try_from(lr_u16).map_err(|_| HmiError::MalformedResponse)?;
            let (ud_u16, sz) = u16::unpack(&src[offset..])?;
            offset += sz;
            let ud = UpDown::try_from(ud_u16).map_err(|_| HmiError::MalformedResponse)?;
            let (fb_u16, sz) = u16::unpack(&src[offset..])?;
            offset += sz;
            let fb = FrontBack::try_from(fb_u16).map_err(|_| HmiError::MalformedResponse)?;
            let (turn4, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (turn5, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (turn6, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            offset += 1;
            let (is_valid, sz) = bool::unpack(&src[offset..])?;
            offset += sz;
            Ok((
                CartesianData {
                    x,
                    y,
                    z,
                    w,
                    p,
                    r,
                    e1,
                    e2,
                    e3,
                    flip,
                    lr,
                    ud,
                    fb,
                    turn4,
                    turn5,
                    turn6,
                    is_valid,
                },
                offset,
            ))
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, set_all, str))]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    pub struct JointData {
        pub j1: f32,
        pub j2: f32,
        pub j3: f32,
        pub j4: f32,
        pub j5: f32,
        pub j6: f32,
        pub j7: f32,
        pub j8: f32,
        pub j9: f32,
        pub is_valid: bool,
    }

    impl std::fmt::Display for JointData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "JointData {{ j1: {}, j2: {}, j3: {}, j4: {}, j5: {}, j6: {}, j7: {}, j8: {}, j9: {}, is_valid: {} }}",
                self.j1,
                self.j2,
                self.j3,
                self.j4,
                self.j5,
                self.j6,
                self.j7,
                self.j8,
                self.j9,
                self.is_valid
            )
        }
    }

    impl HmiWireable for JointData {
        const PACKED_SIZE: usize = 40;
        const SYS_VAR_SIZE: usize = 40;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.j1.pack(&mut dst[offset..]);
            offset += self.j2.pack(&mut dst[offset..]);
            offset += self.j3.pack(&mut dst[offset..]);
            offset += self.j4.pack(&mut dst[offset..]);
            offset += self.j5.pack(&mut dst[offset..]);
            offset += self.j6.pack(&mut dst[offset..]);
            offset += self.j7.pack(&mut dst[offset..]);
            offset += self.j8.pack(&mut dst[offset..]);
            offset += self.j9.pack(&mut dst[offset..]);
            offset += 1;
            offset += self.is_valid.pack(&mut dst[offset..]);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
            let mut offset = 0;
            let (j1, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j2, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j3, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j4, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j5, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j6, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j7, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j8, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            let (j9, sz) = f32::unpack(&src[offset..])?;
            offset += sz;
            offset += 1;
            let (is_valid, sz) = bool::unpack(&src[offset..])?;
            offset += sz;
            Ok((
                JointData {
                    j1,
                    j2,
                    j3,
                    j4,
                    j5,
                    j6,
                    j7,
                    j8,
                    j9,
                    is_valid,
                },
                offset,
            ))
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, set_all, str))]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    pub struct FrameData {
        pub uf: i16,
        pub ut: i16,
        pub(super) reserved: [u16; 3],
    }

    impl std::fmt::Display for FrameData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "FrameData {{ uf: {}, ut: {}, reserved: {:?} }}",
                self.uf, self.ut, self.reserved
            )
        }
    }

    impl HmiWireable for FrameData {
        const PACKED_SIZE: usize = 10;
        const SYS_VAR_SIZE: usize = 10;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.uf.pack(&mut dst[offset..]);
            offset += self.ut.pack(&mut dst[offset..]);
            for i in 0..3 {
                offset += self.reserved[i].pack(&mut dst[offset..]);
            }
            offset
        }
        #[allow(clippy::needless_range_loop)]
        fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
            let mut offset = 0;
            let (uf, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (ut, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let mut reserved = [0u16; 3];
            for i in 0..3 {
                let (val, sz) = u16::unpack(&src[offset..])?;
                offset += sz;
                reserved[i] = val;
            }
            Ok((FrameData { uf, ut, reserved }, offset))
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, set_all, str))]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    pub struct PositionData {
        pub cartesian: CartesianData,
        pub joint: JointData,
        pub frame: FrameData,
    }

    impl std::fmt::Display for PositionData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "PositionData {{ cartesian: {}, joint: {}, frame: {} }}",
                self.cartesian, self.joint, self.frame
            )
        }
    }

    impl HmiWireable for PositionData {
        const PACKED_SIZE: usize = 100;
        const SYS_VAR_SIZE: usize = 100;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.cartesian.pack(&mut dst[offset..]);
            offset += self.joint.pack(&mut dst[offset..]);
            offset += self.frame.pack(&mut dst[offset..]);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), HmiError> {
            let mut offset = 0;
            let (cartesian, sz) = CartesianData::unpack(&src[offset..])?;
            offset += sz;
            let (joint, sz) = JointData::unpack(&src[offset..])?;
            offset += sz;
            let (frame, sz) = FrameData::unpack(&src[offset..])?;
            offset += sz;
            Ok((
                PositionData {
                    cartesian,
                    joint,
                    frame,
                },
                offset,
            ))
        }
    }
}
pub use position_struct::*;

pub mod alarm_struct {
    use int_enum::IntEnum;

    use crate::hmi::asg::HmiWireable;

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(i16)]
    pub enum AlarmSeverity {
        None = 128,
        Warning = 0,
        PauseLocal = 2,
        PauseGlobal = 34,
        StopLocal = 6,
        StopGlobal = 38,
        ServoAlarm = 54,
        AbortLocal = 11,
        AbortGlobal = 43,
        ServoAlarm2 = 58,
        SystemAlarm = 123,
    }

    impl std::fmt::Display for AlarmSeverity {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                AlarmSeverity::None => "None",
                AlarmSeverity::Warning => "Warning",
                AlarmSeverity::PauseLocal => "PauseLocal",
                AlarmSeverity::PauseGlobal => "PauseGlobal",
                AlarmSeverity::StopLocal => "StopLocal",
                AlarmSeverity::StopGlobal => "StopGlobal",
                AlarmSeverity::ServoAlarm => "ServoAlarm",
                AlarmSeverity::AbortLocal => "AbortLocal",
                AlarmSeverity::AbortGlobal => "AbortGlobal",
                AlarmSeverity::ServoAlarm2 => "ServoAlarm2",
                AlarmSeverity::SystemAlarm => "SystemAlarm",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, str))]
    #[derive(Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    pub struct TimeData {
        pub year: i16,
        pub month: i16,
        pub day: i16,
        pub hour: i16,
        pub minute: i16,
        pub second: i16,
    }

    impl std::fmt::Display for TimeData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "TimeData {{ year: {}, month: {}, day: {}, hour: {}, minute: {}, second: {} }}",
                self.year, self.month, self.day, self.hour, self.minute, self.second
            )
        }
    }

    impl HmiWireable for TimeData {
        const PACKED_SIZE: usize = 12;
        const SYS_VAR_SIZE: usize = 12;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.year.pack(&mut dst[offset..]);
            offset += self.month.pack(&mut dst[offset..]);
            offset += self.day.pack(&mut dst[offset..]);
            offset += self.hour.pack(&mut dst[offset..]);
            offset += self.minute.pack(&mut dst[offset..]);
            offset += self.second.pack(&mut dst[offset..]);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), crate::hmi::HmiError> {
            let mut offset = 0;
            let (year, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (month, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (day, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (hour, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (minute, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (second, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            Ok((
                TimeData {
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    second,
                },
                offset,
            ))
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, str))]
    #[derive(Debug, Clone, PartialEq)]
    #[repr(C)]
    pub struct AlarmData {
        pub id: i16,
        pub number: i16,
        pub cause_id: i16,
        pub cause_cnt: i16,
        pub severity: AlarmSeverity,
        pub time: TimeData,
        pub(super) msg: String,
        pub(super) cause_msg: String,
        pub(super) severity_msg: String, // only 18 bytes
    }

    impl std::fmt::Display for AlarmData {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "AlarmData {{ id: {}, number: {}, cause_id: {}, cause_cnt: {}, severity: {}, time: {}, msg: {}, cause_msg: {}, severity_msg: {} }}",
                self.id,
                self.number,
                self.cause_id,
                self.cause_cnt,
                self.severity,
                self.time,
                self.msg,
                self.cause_msg,
                self.severity_msg
            )
        }
    }

    impl HmiWireable for AlarmData {
        const PACKED_SIZE: usize = 200;
        const SYS_VAR_SIZE: usize = 200;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self.id.pack(&mut dst[offset..]);
            offset += self.number.pack(&mut dst[offset..]);
            offset += self.cause_id.pack(&mut dst[offset..]);
            offset += self.cause_cnt.pack(&mut dst[offset..]);
            offset += (self.severity as i16).pack(&mut dst[offset..]);
            offset += self.time.pack(&mut dst[offset..]);
            offset += self.msg.pack(&mut dst[offset..]);
            offset += self.cause_msg.pack(&mut dst[offset..]);
            offset += self
                .severity_msg
                .partial_pack::<80>(0, 18, &mut dst[offset..])
                .unwrap_or(18usize);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), crate::hmi::HmiError> {
            let mut offset = 0;
            let (id, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (number, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (cause_id, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (cause_cnt, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (severity_i16, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let severity = AlarmSeverity::try_from(severity_i16)
                .map_err(|_| crate::hmi::HmiError::MalformedResponse)?;
            let (time, sz) = TimeData::unpack(&src[offset..])?;
            offset += sz;
            let (msg, sz) = String::unpack(&src[offset..])?;
            offset += sz;
            let (cause_msg, sz) = String::unpack(&src[offset..])?;
            offset += sz;
            let (severity_msg, sz) = String::partial_unpack::<80>(&src[offset..], 0, 18)?;
            offset += sz;
            Ok((
                AlarmData {
                    id,
                    number,
                    cause_id,
                    cause_cnt,
                    severity,
                    time,
                    msg,
                    cause_msg,
                    severity_msg,
                },
                offset,
            ))
        }
    }
}
pub use alarm_struct::*;

pub mod prog_status {
    use int_enum::IntEnum;

    use crate::hmi::asg::HmiWireable;

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
    #[derive(Debug, Clone, Copy, PartialEq, Eq, IntEnum)]
    #[repr(u16)]
    pub enum ProgramState {
        Aborted = 0,
        Paused = 1,
        Running = 2,
    }

    impl std::fmt::Display for ProgramState {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            let name = match self {
                ProgramState::Aborted => "Aborted",
                ProgramState::Paused => "Paused",
                ProgramState::Running => "Running",
            };
            write!(f, "{name}")
        }
    }

    #[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, get_all, str))]
    #[derive(Debug, Clone, PartialEq)]
    #[repr(C)]
    pub struct ProgramStatus {
        pub name: String, // 16 chars
        pub line_number: i16,
        pub state: ProgramState,
        pub parent_name: String, // 16 chars
    }

    impl std::fmt::Display for ProgramStatus {
        fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
            write!(
                f,
                "ProgramStatus {{ name: {}, line_number: {}, state: {}, parent_name: {} }}",
                self.name, self.line_number, self.state, self.parent_name
            )
        }
    }

    impl HmiWireable for ProgramStatus {
        const PACKED_SIZE: usize = 36;
        const SYS_VAR_SIZE: usize = 36;
        fn pack(&self, dst: &mut [u8]) -> usize {
            let mut offset = 0;
            offset += self
                .name
                .partial_pack::<80>(0, 16, &mut dst[offset..])
                .unwrap_or(16usize);
            offset += self.line_number.pack(&mut dst[offset..]);
            offset += (self.state as u16).pack(&mut dst[offset..]);
            offset += self
                .parent_name
                .partial_pack::<80>(0, 16, &mut dst[offset..])
                .unwrap_or(16usize);
            offset
        }
        fn unpack(src: &[u8]) -> Result<(Self, usize), crate::hmi::HmiError> {
            let mut offset = 0;
            let (name, sz) = String::partial_unpack::<80>(&src[offset..], 0, 16)?;
            offset += sz;
            let (line_number, sz) = i16::unpack(&src[offset..])?;
            offset += sz;
            let (state_u16, sz) = u16::unpack(&src[offset..])?;
            offset += sz;
            let state = ProgramState::try_from(state_u16)
                .map_err(|_| crate::hmi::HmiError::MalformedResponse)?;
            let (parent_name, sz) = String::partial_unpack::<80>(&src[offset..], 0, 16)?;
            offset += sz;
            Ok((
                ProgramStatus {
                    name,
                    line_number,
                    state,
                    parent_name,
                },
                offset,
            ))
        }
    }
}
pub use prog_status::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsgTypeMismatchError {
    pub expected: AsgTag,
    pub found: AsgTag,
}
impl std::fmt::Display for AsgTypeMismatchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ASG type mismatch: expected {:?}, found {:?}",
            self.expected, self.found
        )
    }
}
impl std::error::Error for AsgTypeMismatchError {}

pub trait AsgEncodableType:
    Into<AsgValue>
    + TryFrom<AsgValue, Error = AsgTypeMismatchError>
    + HmiWireable
    + Sized
    + Send
    + Sync
    + 'static
    + Clone
    + std::fmt::Debug
{
    const TAG: AsgTag;
    const REGISTER_CNT: u16;
    fn from_message(msg: Message, offset: u16, member_count: u16) -> HmiResult<Self> {
        let payload = msg.payload();
        let (value, _) = if 2 > offset {
            log::debug!("Unpacking payload of size {} as full sysvar", payload.len());
            Self::unpack_sysvar(payload)?
        } else {
            Self::partial_unpack_sysvar::<80>(payload, offset as usize, member_count as usize)?
        };
        Ok(value)
    }
    fn many_from_message<const N: usize>(
        msg: Message,
        offset: u16,
        member_count: u16,
    ) -> HmiResult<[Self; N]> {
        let item_count = N;
        let payload = msg.payload();
        let mut ret = Vec::with_capacity(item_count);
        let byte_count = member_count as usize * 2;
        log::trace!(
            "Decoding {} ASG items with {} members each",
            item_count,
            member_count
        );
        for i in 0..item_count {
            let item_start = i * byte_count;
            let item_payload = &payload[item_start..item_start + byte_count];
            let (value, _) = if 2 > offset {
                Self::unpack_sysvar(item_payload)?
            } else {
                Self::partial_unpack_sysvar::<80>(item_payload, offset as usize, byte_count)?
            };
            ret.push(value);
        }
        ret.try_into().map_err(|_| {
            log::error!("Failed to convert Vec to array of size {}", N);
            HmiError::MalformedResponse
        })
    }
}

pub trait SysVarVal: AsgEncodableType {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsgTag {
    Integer,
    Short,
    Byte,
    Real,
    Bool,
    String,
    Position,
    Alarm,
    Program,
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Debug, Clone)]
pub enum AsgValue {
    Integer(i32),
    Short(i16),
    Byte(i8),
    Real(f32),
    Bool(bool),
    String(String),
    Position(position_struct::PositionData),
    Alarm(alarm_struct::AlarmData),
    Program(prog_status::ProgramStatus),
}

impl AsgValue {
    pub fn tag(&self) -> AsgTag {
        match self {
            AsgValue::Integer(_) => AsgTag::Integer,
            AsgValue::Short(_) => AsgTag::Short,
            AsgValue::Byte(_) => AsgTag::Byte,
            AsgValue::Real(_) => AsgTag::Real,
            AsgValue::Bool(_) => AsgTag::Bool,
            AsgValue::String(_) => AsgTag::String,
            AsgValue::Position(_) => AsgTag::Position,
            AsgValue::Alarm(_) => AsgTag::Alarm,
            AsgValue::Program(_) => AsgTag::Program,
        }
    }
}

macro_rules! def_asg_type {
    ($variant:ident<$typ:ty>[$reg_cnt:expr]) => {
        impl From<$typ> for AsgValue {
            fn from(value: $typ) -> Self {
                AsgValue::$variant(value)
            }
        }

        impl TryFrom<AsgValue> for $typ {
            type Error = AsgTypeMismatchError;

            fn try_from(value: AsgValue) -> Result<Self, Self::Error> {
                if let AsgValue::$variant(inner) = value {
                    Ok(inner)
                } else {
                    Err(AsgTypeMismatchError {
                        expected: AsgTag::$variant,
                        found: value.tag(),
                    })
                }
            }
        }

        impl AsgEncodableType for $typ {
            const TAG: AsgTag = AsgTag::$variant;
            const REGISTER_CNT: u16 = $reg_cnt;
        }
    };
}

def_asg_type!(Integer<i32>[2]);
def_asg_type!(Short<i16>[2]);
def_asg_type!(Byte<i8>[2]);
def_asg_type!(Real<f32>[2]);
def_asg_type!(Bool<bool>[2]);
def_asg_type!(String<String>[40]);
def_asg_type!(Position<position_struct::PositionData>[50]);
def_asg_type!(Alarm<alarm_struct::AlarmData>[100]);
def_asg_type!(Program<prog_status::ProgramStatus>[18]);

impl SysVarVal for i32 {}
impl SysVarVal for i16 {}
impl SysVarVal for i8 {}
impl SysVarVal for f32 {}
impl SysVarVal for bool {}
impl SysVarVal for String {}
impl SysVarVal for position_struct::PositionData {}

pub const MAX_ASG_CNT: usize = 80;

#[derive(Debug, Clone, PartialEq)]
pub struct AsgEntry {
    pub address: u16,
    pub size: u16,
    pub var_name: String,
    pub var_slice_range: (u16, u16),
    pub multiply: f32,
    pub tag: AsgTag,
}

impl Hash for AsgEntry {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.address.hash(state);
        self.size.hash(state);
        self.var_name.hash(state);
        self.var_slice_range.hash(state);
        ((self.multiply * 10_000.0) as u64).hash(state);
    }
}

impl Display for AsgEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "AsgEntry {{ address: {}, size: {}, var_name: {}, var_slice_range: {:?}, multiply: {}, tag: {:?} }}",
            self.address, self.size, self.var_name, self.var_slice_range, self.multiply, self.tag
        )
    }
}

impl AsgEntry {
    pub fn encode_to_bytes(&self, value: &AsgValue) -> HmiResult<Vec<u8>> {
        if value.tag() != self.tag {
            return Err(HmiError::Other("Incompatible ASG value".into()));
        }
        Ok(match value {
            AsgValue::Integer(v) => {
                let mut buf = vec![0; i32::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Short(v) => {
                let mut buf = vec![0; i16::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Byte(v) => {
                let mut buf = vec![0; i8::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Real(v) => {
                let mut buf = vec![0; f32::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Bool(v) => {
                let mut buf = vec![0; bool::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::String(v) => {
                let mut buf = vec![0; String::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Position(v) => {
                let mut buf = vec![0; position_struct::PositionData::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Alarm(v) => {
                let mut buf = vec![0; alarm_struct::AlarmData::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
            AsgValue::Program(v) => {
                let mut buf = vec![0; prog_status::ProgramStatus::SYS_VAR_SIZE];
                v.pack_sysvar(&mut buf);
                buf
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::hmi::asg::position_struct::PositionData;

    use super::*;
    #[test]
    fn test_position_data_size() {
        let pos_ref = PositionData {
            cartesian: position_struct::CartesianData {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
                p: 0.0,
                r: 0.0,
                e1: 0.0,
                e2: 0.0,
                e3: 0.0,
                flip: position_struct::FlipState::Unflipped,
                lr: position_struct::LeftRight::Right,
                ud: position_struct::UpDown::Down,
                fb: position_struct::FrontBack::Back,
                turn4: 0,
                turn5: 0,
                turn6: 0,
                is_valid: true,
            },
            joint: position_struct::JointData {
                j1: 0.0,
                j2: 0.0,
                j3: 0.0,
                j4: 0.0,
                j5: 0.0,
                j6: 0.0,
                j7: 0.0,
                j8: 0.0,
                j9: 0.0,
                is_valid: true,
            },
            frame: position_struct::FrameData {
                uf: 0,
                ut: 0,
                reserved: [0; 3],
            },
        };

        let mut bytes = vec![0u8; PositionData::SYS_VAR_SIZE];
        pos_ref.pack_sysvar(&mut bytes);
        assert_eq!(bytes.len(), PositionData::REGISTER_CNT as usize * 2);
    }

    #[test]
    fn test_alarm_data_size() {
        use crate::hmi::asg::alarm_struct::{AlarmData, AlarmSeverity, TimeData};

        let alarm_ref = AlarmData {
            id: 1,
            number: 100,
            cause_id: 200,
            cause_cnt: 1,
            severity: AlarmSeverity::StopLocal,
            time: TimeData {
                year: 0,
                month: 0,
                day: 0,
                hour: 0,
                minute: 0,
                second: 0,
            },
            msg: String::from("test msg"),
            cause_msg: String::from("cause msg"),
            severity_msg: String::from("severity msg"),
        };

        let mut bytes = vec![0u8; AlarmData::SYS_VAR_SIZE];
        alarm_ref.pack_sysvar(&mut bytes);
        assert_eq!(bytes.len(), AlarmData::REGISTER_CNT as usize * 2);
    }

    #[test]
    fn test_program_status_size() {
        use crate::hmi::asg::prog_status::{ProgramState, ProgramStatus};

        let prog_ref = ProgramStatus {
            name: String::from("TestProgram"),
            line_number: 10,
            state: ProgramState::Running,
            parent_name: String::from("ParentProgram"),
        };

        let mut bytes = vec![0u8; ProgramStatus::SYS_VAR_SIZE];
        prog_ref.pack_sysvar(&mut bytes);
        assert_eq!(bytes.len(), ProgramStatus::REGISTER_CNT as usize * 2);
        let (unp_prog_ref, _) = ProgramStatus::unpack_sysvar(&bytes).unwrap();
        assert_eq!(prog_ref.line_number, unp_prog_ref.line_number);
    }

    #[test]
    fn bytes_to_i16_empty_input_yields_empty_output() {
        let out = bytes_to_i16(&[]);
        assert!(out.is_empty());
    }

    #[test]
    fn bytes_to_i16_decodes_little_endian() {
        // Little-endian: 0x01 0x00 -> 1, 0xFF 0xFF -> -1, 0x80 0x00 -> 128.
        let bytes = [0x01, 0x00, 0xFF, 0xFF, 0x80, 0x00];
        let out = bytes_to_i16(&bytes);
        assert_eq!(out, vec![1i16, -1, 128]);
    }

    #[test]
    fn bytes_to_i16_drops_unpaired_trailing_byte() {
        // chunks_exact(2) discards the dangling last byte rather than
        // panicking — the encode side is expected to always be even-aligned,
        // and dropping is preferable to UB on misaligned reads (the original
        // unsafe `from_raw_parts` had no such guard).
        let bytes = [0x01, 0x00, 0xFE]; // 3 bytes
        let out = bytes_to_i16(&bytes);
        assert_eq!(out, vec![1i16]);
    }

    #[test]
    fn bytes_to_i16_round_trips_against_to_le_bytes() {
        let values: Vec<i16> = vec![0, 1, -1, 32767, -32768, 256, -256, 1234, -4321];
        let bytes: Vec<u8> = values.iter().flat_map(|v| v.to_le_bytes()).collect();
        let out = bytes_to_i16(&bytes);
        assert_eq!(out, values);
    }

    #[test]
    fn bytes_to_i16_works_on_unaligned_slice() {
        // The original unsafe impl required the source pointer to be 2-byte
        // aligned to satisfy `slice::from_raw_parts`'s contract, otherwise
        // the read of `i16` was UB. A `&[u8]` from a sub-slice starting at
        // an odd offset is NOT guaranteed aligned for `*const i16`; the
        // safe `from_le_bytes` impl must handle this without UB or panic.
        let owned = vec![0xAA, 0x01, 0x00, 0xFF, 0xFF];
        let unaligned = &owned[1..]; // starts at offset 1, length 4
        let out = bytes_to_i16(unaligned);
        assert_eq!(out, vec![1i16, -1]);
    }
}
