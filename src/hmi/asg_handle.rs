use std::sync::Arc;

use crate::hmi::{
    DriverResult, HmiDriver, HmiHandle,
    asg::{self, AsgEncodableType, AsgEntry, bytes_to_i16},
    proto::ports,
};

/// A typed interface for reading and writing a registered ASG variable on the controller.
///
/// `T` is the value type and `ARR` is the array size (1 for scalar variables).
#[derive(Debug, Clone, PartialEq)]
pub struct AsgVarInterface<T: AsgEncodableType, const ARR: usize = 1> {
    entry: Arc<AsgEntry>,
    _marker: std::marker::PhantomData<T>,
}

impl<T: AsgEncodableType, const ARR: usize> AsgVarInterface<T, ARR> {
    pub(crate) fn new(entry: Arc<AsgEntry>) -> Self {
        Self {
            entry,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<T: AsgEncodableType> AsgVarInterface<T, 1> {
    /// Writes a value to this ASG variable on the controller, returning a handle to the asynchronous response.
    pub fn write(
        &self,
        driver: &HmiDriver,
        value: impl AsgEncodableType,
    ) -> DriverResult<HmiHandle<()>> {
        let bytes = self.entry.encode_to_bytes(&value.into())?;
        driver.write_array::<ports::Register>(self.entry.address as usize, bytes_to_i16(&bytes))
    }

    /// Reads the current value of this ASG variable from the controller, returning a handle to the asynchronous response.
    pub fn read(&self, driver: &HmiDriver) -> DriverResult<HmiHandle<T>> {
        let gen_handle = driver
            .read_array::<ports::Register>(
                self.entry.address as usize,
                self.entry.var_slice_range.1 as usize,
            )?
            .generic();
        Ok(HmiHandle::new_from_generic(
            &gen_handle,
            self.entry.var_slice_range.0,
            self.entry.var_slice_range.1,
            T::from_message,
        ))
    }
}

macro_rules! arr_size_impl {
    { $($num:literal),* } => {
        $(
            impl<T: AsgEncodableType> AsgVarInterface<T, $num> {
                pub fn write(
                    &self,
                    driver: &HmiDriver,
                    values: [impl AsgEncodableType; $num],
                ) -> DriverResult<HmiHandle<()>> {
                    let mut bytes = Vec::with_capacity((T::REGISTER_CNT * 2) as usize);
                    for value in values.into_iter() {
                        bytes.extend(self.entry.encode_to_bytes(&value.into())?);
                    }
                    driver.write_array::<ports::Register>(self.entry.address as usize, bytes_to_i16(&bytes))
                }

                pub fn read(&self, driver: &HmiDriver) -> DriverResult<HmiHandle<[T; $num]>> {
                    let gen_handle = driver
                        .read_array::<ports::Register>(
                            self.entry.address as usize,
                            self.entry.var_slice_range.1 as usize,
                        )?
                        .generic();
                    Ok(HmiHandle::new_from_generic(
                        &gen_handle,
                        self.entry.var_slice_range.0,
                        self.entry.var_slice_range.1,
                        T::many_from_message::<$num>,
                    ))
                }
            }
        )*
    };
}

arr_size_impl! {
    2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99, 100
}

/// Trait for types that describe an ASG variable to be registered on the controller.
///
/// Implementors define how to produce an [`AsgEntry`] from their configuration fields.
pub trait AsgArgument: Sized + Send + Sync + 'static {
    type Ret: AsgEncodableType;
    const WRITABLE: bool = true;
    fn size_in_registers(&self) -> u16;
    fn to_asg_entry(self) -> AsgEntry;
}

/// Arguments for registering a position register (`PR[...]`) as an ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct PosRegArgs {
    pub index: u16,
    pub group: Option<u8>,
    pub range: Option<(u16, u16)>,
}
impl Default for PosRegArgs {
    fn default() -> Self {
        Self {
            index: 1,
            group: None,
            range: None,
        }
    }
}
impl AsgArgument for PosRegArgs {
    type Ret = asg::position_struct::PositionData;
    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, length)) => length,
            None => asg::position_struct::PositionData::REGISTER_CNT,
        }
    }
    fn to_asg_entry(self) -> AsgEntry {
        let g_str = match self.group {
            Some(g) => format!("G{}:", g),
            None => "".to_string(),
        };
        let r_str = match self.range {
            Some((offset, length)) => format!("@{}.{}", offset, length),
            None => "".to_string(),
        };
        AsgEntry {
            var_name: format!("PR[{}{}]{}", g_str, self.index, r_str),
            tag: asg::AsgTag::Position,
            address: 0, // to be filled in by the driver
            size: self.size_in_registers(),
            var_slice_range: match self.range {
                Some((offset, length)) => (offset, length),
                None => (0, asg::position_struct::PositionData::REGISTER_CNT),
            },
            multiply: 0.0,
        }
    }
}

/// Arguments for registering the current position (`POS[...]`) as a read-only ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct CurPosArgs {
    pub frame: i8,
    pub group: Option<u8>,
    pub range: Option<(u16, u16)>,
}
impl Default for CurPosArgs {
    fn default() -> Self {
        Self {
            frame: 0,
            group: None,
            range: None,
        }
    }
}
impl AsgArgument for CurPosArgs {
    type Ret = asg::position_struct::PositionData;
    const WRITABLE: bool = false;
    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, length)) => length,
            None => asg::position_struct::PositionData::REGISTER_CNT,
        }
    }
    fn to_asg_entry(self) -> AsgEntry {
        let g_str = match self.group {
            Some(g) => format!("G{}:", g),
            None => "".to_string(),
        };
        let r_str = match self.range {
            Some((offset, length)) => format!("@{}.{}", offset, length),
            None => "".to_string(),
        };
        AsgEntry {
            var_name: format!("POS[{}{}]{}", g_str, self.frame, r_str),
            tag: asg::AsgTag::Position,
            address: 0, // to be filled in by the driver
            size: self.size_in_registers(),
            var_slice_range: match self.range {
                Some((offset, length)) => (offset, length),
                None => (0, asg::position_struct::PositionData::REGISTER_CNT),
            },
            multiply: 0.0,
        }
    }
}

/// Arguments for registering a numeric register (`R[...]`) as an ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct NumRegArgs {
    pub index: u16,
    pub range: Option<(u16, u16)>,
}

impl Default for NumRegArgs {
    fn default() -> Self {
        Self {
            index: 1,
            range: None,
        }
    }
}

impl AsgArgument for NumRegArgs {
    type Ret = f32;

    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, len)) => len,
            None => <Self::Ret as AsgEncodableType>::REGISTER_CNT,
        }
    }

    fn to_asg_entry(self) -> AsgEntry {
        let default_len = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let (offset, len, suffix) = format_slice_suffix(default_len, self.range);
        AsgEntry {
            var_name: format!("R[{}]{}", self.index, suffix),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: len,
            var_slice_range: (offset, len),
            multiply: 0.0,
        }
    }
}

fn format_slice_suffix(default_len: u16, range: Option<(u16, u16)>) -> (u16, u16, String) {
    match range {
        Some((offset, len)) => (offset, len, format!("@{}.{}", offset, len)),
        None => (0, default_len, String::new()),
    }
}

/// Arguments for registering a string register (`SR[...]`) as an ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct StringRegArgs {
    pub index: u16,
    pub range: Option<(u16, u16)>,
}

impl Default for StringRegArgs {
    fn default() -> Self {
        Self {
            index: 1,
            range: None,
        }
    }
}

impl AsgArgument for StringRegArgs {
    type Ret = String;
    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, len)) => len,
            None => <Self::Ret as AsgEncodableType>::REGISTER_CNT,
        }
    }

    fn to_asg_entry(self) -> AsgEntry {
        let default_len = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let (offset, len, suffix) = format_slice_suffix(default_len, self.range);
        AsgEntry {
            var_name: format!("SR[{}]{}", self.index, suffix),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: len,
            var_slice_range: (offset, len),
            multiply: 1.0,
        }
    }
}

/// Boolean I/O signal types available on a FANUC controller.
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoolIoSignal {
    DigitalInput,
    DigitalOutput,
    RobotInput,
    RobotOutput,
    UopInput,
    UopOutput,
    SopInput,
    SopOutput,
    WeldInput,
    WeldOutput,
    WireStickInput,
    WireStickOutput,
}

impl BoolIoSignal {
    fn as_str(&self) -> &'static str {
        match self {
            BoolIoSignal::DigitalInput => "DI",
            BoolIoSignal::DigitalOutput => "DO",
            BoolIoSignal::RobotInput => "RI",
            BoolIoSignal::RobotOutput => "RO",
            BoolIoSignal::UopInput => "UI",
            BoolIoSignal::UopOutput => "UO",
            BoolIoSignal::SopInput => "SI",
            BoolIoSignal::SopOutput => "SO",
            BoolIoSignal::WeldInput => "WI",
            BoolIoSignal::WeldOutput => "WO",
            BoolIoSignal::WireStickInput => "WSI",
            BoolIoSignal::WireStickOutput => "WSO",
        }
    }
}

impl std::fmt::Display for BoolIoSignal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BoolIoSignal::DigitalInput => write!(f, "DigitalInput"),
            BoolIoSignal::DigitalOutput => write!(f, "DigitalOutput"),
            BoolIoSignal::RobotInput => write!(f, "RobotInput"),
            BoolIoSignal::RobotOutput => write!(f, "RobotOutput"),
            BoolIoSignal::UopInput => write!(f, "UopInput"),
            BoolIoSignal::UopOutput => write!(f, "UopOutput"),
            BoolIoSignal::SopInput => write!(f, "SopInput"),
            BoolIoSignal::SopOutput => write!(f, "SopOutput"),
            BoolIoSignal::WeldInput => write!(f, "WeldInput"),
            BoolIoSignal::WeldOutput => write!(f, "WeldOutput"),
            BoolIoSignal::WireStickInput => write!(f, "WireStickInput"),
            BoolIoSignal::WireStickOutput => write!(f, "WireStickOutput"),
        }
    }
}

/// Arguments for registering a boolean I/O signal as an ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct BoolIoArgs {
    pub signal: BoolIoSignal,
    pub index: u16,
    pub simulation: bool,
}

impl Default for BoolIoArgs {
    fn default() -> Self {
        Self {
            signal: BoolIoSignal::DigitalInput,
            index: 1,
            simulation: false,
        }
    }
}

impl AsgArgument for BoolIoArgs {
    type Ret = bool;

    fn size_in_registers(&self) -> u16 {
        <Self::Ret as AsgEncodableType>::REGISTER_CNT
    }

    fn to_asg_entry(self) -> AsgEntry {
        let register_cnt = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let idx_fragment = if self.simulation {
            format!("S{}", self.index)
        } else {
            self.index.to_string()
        };
        AsgEntry {
            var_name: format!("{}[{}]", self.signal.as_str(), idx_fragment),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: register_cnt,
            var_slice_range: (0, register_cnt),
            multiply: 1.0,
        }
    }
}

/// Integer (group/analog) I/O signal types available on a FANUC controller.
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntIoSignal {
    GroupInput,
    GroupOutput,
    AnalogInput,
    AnalogOutput,
}

impl IntIoSignal {
    fn as_str(&self) -> &'static str {
        match self {
            IntIoSignal::GroupInput => "GI",
            IntIoSignal::GroupOutput => "GO",
            IntIoSignal::AnalogInput => "AI",
            IntIoSignal::AnalogOutput => "AO",
        }
    }
}

impl std::fmt::Display for IntIoSignal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IntIoSignal::GroupInput => write!(f, "GroupInput"),
            IntIoSignal::GroupOutput => write!(f, "GroupOutput"),
            IntIoSignal::AnalogInput => write!(f, "AnalogInput"),
            IntIoSignal::AnalogOutput => write!(f, "AnalogOutput"),
        }
    }
}

/// Arguments for registering an integer I/O signal as an ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct IntIoArgs {
    pub signal: IntIoSignal,
    pub index: u16,
    pub simulation: bool,
}

impl Default for IntIoArgs {
    fn default() -> Self {
        Self {
            signal: IntIoSignal::GroupInput,
            index: 1,
            simulation: false,
        }
    }
}

impl AsgArgument for IntIoArgs {
    type Ret = i16;

    fn size_in_registers(&self) -> u16 {
        <Self::Ret as AsgEncodableType>::REGISTER_CNT
    }

    fn to_asg_entry(self) -> AsgEntry {
        let register_cnt = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let idx_fragment = if self.simulation {
            format!("S{}", self.index)
        } else {
            self.index.to_string()
        };
        AsgEntry {
            var_name: format!("{}[{}]", self.signal.as_str(), idx_fragment),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: register_cnt,
            var_slice_range: (0, register_cnt),
            multiply: 1.0,
        }
    }
}

/// Source log from which to read alarms on the controller.
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmSource {
    Active,
    History,
    PasswordLog,
    Motion,
    Application,
    System,
}

impl Default for AlarmSource {
    fn default() -> Self {
        AlarmSource::Active
    }
}

impl AlarmSource {
    fn prefix(&self) -> &'static str {
        match self {
            AlarmSource::Active => "",
            AlarmSource::History => "E",
            AlarmSource::PasswordLog => "P",
            AlarmSource::Motion => "M",
            AlarmSource::Application => "A",
            AlarmSource::System => "S",
        }
    }
}

impl std::fmt::Display for AlarmSource {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AlarmSource::Active => write!(f, "Active"),
            AlarmSource::History => write!(f, "History"),
            AlarmSource::PasswordLog => write!(f, "PasswordLog"),
            AlarmSource::Motion => write!(f, "Motion"),
            AlarmSource::Application => write!(f, "Application"),
            AlarmSource::System => write!(f, "System"),
        }
    }
}

/// Arguments for registering an alarm entry as a read-only ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct AlarmArgs {
    pub source: AlarmSource,
    pub line: u16,
    pub range: Option<(u16, u16)>,
}

impl Default for AlarmArgs {
    fn default() -> Self {
        Self {
            source: AlarmSource::default(),
            line: 1,
            range: None,
        }
    }
}

impl AsgArgument for AlarmArgs {
    type Ret = asg::alarm_struct::AlarmData;
    const WRITABLE: bool = false;

    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, len)) => len,
            None => <Self::Ret as AsgEncodableType>::REGISTER_CNT,
        }
    }

    fn to_asg_entry(self) -> AsgEntry {
        let default_len = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let (offset, len, suffix) = format_slice_suffix(default_len, self.range);
        AsgEntry {
            var_name: format!("ALM[{}{}]{}", self.source.prefix(), self.line, suffix),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: len,
            var_slice_range: (offset, len),
            multiply: 1.0,
        }
    }
}

/// The kind of program task to query status for.
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object, str))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgramStatusKind {
    Default,
    MacroCaller,
    KarelCaller,
    MacroOrKarel,
}

impl Default for ProgramStatusKind {
    fn default() -> Self {
        ProgramStatusKind::Default
    }
}

impl ProgramStatusKind {
    fn prefix(&self) -> &'static str {
        match self {
            ProgramStatusKind::Default => "",
            ProgramStatusKind::MacroCaller => "M",
            ProgramStatusKind::KarelCaller => "K",
            ProgramStatusKind::MacroOrKarel => "MK",
        }
    }
}

impl std::fmt::Display for ProgramStatusKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProgramStatusKind::Default => write!(f, "Default"),
            ProgramStatusKind::MacroCaller => write!(f, "MacroCaller"),
            ProgramStatusKind::KarelCaller => write!(f, "KarelCaller"),
            ProgramStatusKind::MacroOrKarel => write!(f, "MacroOrKarel"),
        }
    }
}

/// Arguments for registering a program status entry as a read-only ASG variable.
#[derive(Debug, Clone, Copy)]
pub struct ProgramStatusArgs {
    pub task: u16,
    pub kind: ProgramStatusKind,
    pub range: Option<(u16, u16)>,
}

impl Default for ProgramStatusArgs {
    fn default() -> Self {
        Self {
            task: 1,
            kind: ProgramStatusKind::default(),
            range: None,
        }
    }
}

impl AsgArgument for ProgramStatusArgs {
    type Ret = asg::prog_status::ProgramStatus;
    const WRITABLE: bool = false;

    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, len)) => len,
            None => <Self::Ret as AsgEncodableType>::REGISTER_CNT,
        }
    }

    fn to_asg_entry(self) -> AsgEntry {
        let default_len = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let (offset, len, suffix) = format_slice_suffix(default_len, self.range);
        AsgEntry {
            var_name: format!("PRG[{}{}]{}", self.kind.prefix(), self.task, suffix),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: len,
            var_slice_range: (offset, len),
            multiply: 1.0,
        }
    }
}

/// Arguments for registering an arbitrary system variable by name as an ASG variable.
#[derive(Debug, Clone)]
pub struct SysVarArgs<T: asg::SysVarVal> {
    pub var_name: String,
    pub range: Option<(u16, u16)>,
    pub _marker: std::marker::PhantomData<T>,
}

impl<T: asg::SysVarVal> Default for SysVarArgs<T> {
    fn default() -> Self {
        Self {
            var_name: String::new(),
            range: None,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<T: asg::SysVarVal> AsgArgument for SysVarArgs<T> {
    type Ret = T;

    fn size_in_registers(&self) -> u16 {
        match self.range {
            Some((_, len)) => len,
            None => <Self::Ret as AsgEncodableType>::REGISTER_CNT,
        }
    }

    fn to_asg_entry(self) -> AsgEntry {
        let default_len = <Self::Ret as AsgEncodableType>::REGISTER_CNT;
        let (offset, len, suffix) = format_slice_suffix(default_len, self.range);
        let multiply = match T::TAG {
            asg::AsgTag::Integer
            | asg::AsgTag::Short
            | asg::AsgTag::Byte
            | asg::AsgTag::Real
            | asg::AsgTag::Position => 0.0,
            _ => 1.0,
        };
        AsgEntry {
            var_name: format!("{}{}", self.var_name, suffix),
            tag: <Self::Ret as AsgEncodableType>::TAG,
            address: 0,
            size: len,
            var_slice_range: (offset, len),
            multiply,
        }
    }
}

#[cfg(feature = "py")]
pub(super) mod py {

    use crate::hmi::{
        asg::{AsgTag, AsgValue},
        hmi_handle::py::PyHmiHandleGeneric,
        proto::asg::{alarm_struct, position_struct, prog_status},
    };

    use super::*;
    use pyo3::{IntoPyObjectExt, prelude::*, pyclass, pymethods};

    macro_rules! _arr_size_impl {
        (
            $cls:ty;
            $item_count:ident;
            , $($num:literal),*
        ) => {
                if $item_count == 1 {
                    asg_py_caster::<$cls>
                }
                $(
                    else if $item_count == $num {
                        asg_array_py_caster::<$cls, $num>
                    }
                )*
                else {
                    return Err(pyo3::exceptions::PyValueError::new_err("Unsupported array size for Asg array caster"))
                }
        };
    }

    macro_rules! asg_generic_switch {
        (
            $cls:ty;
            $item_count:ident
        ) => {
            _arr_size_impl! {
                $cls;
                $item_count;
                , 2, 3, 4, 5, 6, 7, 8, 9, 10,
                  11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
                  21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
                  31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
                  41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
                  51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
                  61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
                  71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
                  81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
                  91, 92, 93, 94, 95, 96, 97, 98, 99, 100
            }
        };
    }

    fn asg_py_caster<'a, T: AsgEncodableType>(
        py: Python<'a>,
        msg: crate::hmi::proto::wire::Message,
        offset: u16,
        member_count: u16,
    ) -> pyo3::PyResult<pyo3::Bound<'a, pyo3::types::PyAny>> {
        let value: AsgValue = T::from_message(msg, offset, member_count).map(Into::into)?;
        asg_value_to_py_value(py, &value)
    }

    fn asg_array_py_caster<'a, T: AsgEncodableType, const N: usize>(
        py: Python<'a>,
        msg: crate::hmi::proto::wire::Message,
        offset: u16,
        member_count: u16,
    ) -> pyo3::PyResult<pyo3::Bound<'a, pyo3::types::PyAny>> {
        if N < 1 || N > 100 {
            return Err(pyo3::exceptions::PyValueError::new_err(
                "Unsupported array size for Asg array caster",
            ));
        }
        let values: [T; N] = T::many_from_message::<N>(msg, offset, member_count)?;
        let list = pyo3::types::PyList::empty(py);
        for v in values.into_iter() {
            let obj = asg_value_to_py_value(py, &v.into())?;
            list.append(obj)?;
        }
        Ok(list.into_bound_py_any(py)?)
    }

    fn caster_null<'a>(
        py: Python<'a>,
        _msg: crate::hmi::proto::wire::Message,
        _offset: u16,
        _member_count: u16,
    ) -> pyo3::PyResult<pyo3::Bound<'a, pyo3::types::PyAny>> {
        py.None().into_bound_py_any(py)
    }

    fn py_value_to_asg_value(
        value: &Bound<'_, pyo3::types::PyAny>,
        tag: AsgTag,
    ) -> PyResult<AsgValue> {
        Ok(match tag {
            AsgTag::Integer => AsgValue::Integer(value.extract::<i32>()?),
            AsgTag::Short => AsgValue::Short(value.extract::<i16>()?),
            AsgTag::Byte => AsgValue::Byte(value.extract::<i8>()?),
            AsgTag::Real => AsgValue::Real(value.extract::<f32>()?),
            AsgTag::Bool => AsgValue::Bool(value.extract::<bool>()?),
            AsgTag::String => AsgValue::String(value.extract::<String>()?),
            AsgTag::Position => {
                AsgValue::Position(value.extract::<position_struct::PositionData>()?)
            }
            AsgTag::Alarm => AsgValue::Alarm(value.extract::<alarm_struct::AlarmData>()?),
            AsgTag::Program => AsgValue::Program(value.extract::<prog_status::ProgramStatus>()?),
        })
    }

    fn asg_value_to_py_value<'a>(
        py: Python<'a>,
        asg_value: &AsgValue,
    ) -> PyResult<Bound<'a, pyo3::types::PyAny>> {
        match asg_value {
            AsgValue::Integer(v) => v.into_bound_py_any(py),
            AsgValue::Short(v) => v.into_bound_py_any(py),
            AsgValue::Byte(v) => v.into_bound_py_any(py),
            AsgValue::Real(v) => v.into_bound_py_any(py),
            AsgValue::Bool(v) => v.into_bound_py_any(py),
            AsgValue::String(v) => v.into_bound_py_any(py),
            AsgValue::Position(v) => v.into_bound_py_any(py),
            AsgValue::Alarm(v) => v.clone().into_bound_py_any(py),
            AsgValue::Program(v) => v.clone().into_bound_py_any(py),
        }
    }

    #[derive(Debug)]
    #[pyclass(name = "AsgInterface")]
    pub struct PyAsgVarInterface {
        entry: Arc<AsgEntry>,
        count: usize,
        can_write: bool,
    }

    impl PyAsgVarInterface {
        pub(crate) fn new(entry: Arc<AsgEntry>, count: usize, can_write: bool) -> Self {
            Self {
                entry,
                count,
                can_write,
            }
        }
    }

    #[pymethods]
    impl PyAsgVarInterface {
        pub fn write(
            &self,
            driver: &HmiDriver,
            value: Bound<'_, pyo3::types::PyAny>,
        ) -> DriverResult<PyHmiHandleGeneric> {
            if !self.can_write {
                return Err(pyo3::exceptions::PyPermissionError::new_err(
                    "This ASG variable is read-only",
                )
                .into());
            }
            let asg_value = py_value_to_asg_value(&value, self.entry.tag)?;
            let bytes = self.entry.encode_to_bytes(&asg_value)?;
            let gen_handle = driver
                .write_array::<ports::Register>(self.entry.address as usize, bytes_to_i16(&bytes))?
                .generic();
            Ok(PyHmiHandleGeneric {
                inner: gen_handle,
                target: self.entry.var_slice_range.0,
                count: self.entry.var_slice_range.1,
                caster: caster_null,
            })
        }

        pub fn read(&self, driver: &HmiDriver) -> DriverResult<PyHmiHandleGeneric> {
            let gen_handle = driver
                .read_array::<ports::Register>(
                    self.entry.address as usize,
                    self.entry.size as usize,
                )?
                .generic();
            let item_count = self.count;
            let caster = match self.entry.tag {
                AsgTag::Integer => asg_generic_switch!(i32; item_count),
                AsgTag::Short => asg_generic_switch!(i16; item_count),
                AsgTag::Real => asg_generic_switch!(f32; item_count),
                AsgTag::Byte => asg_generic_switch!(i8; item_count),
                AsgTag::Bool => asg_generic_switch!(bool; item_count),
                AsgTag::String => asg_generic_switch!(String; item_count),
                AsgTag::Position => asg_generic_switch!(position_struct::PositionData; item_count),
                AsgTag::Alarm => asg_generic_switch!(alarm_struct::AlarmData; item_count),
                AsgTag::Program => asg_generic_switch!(prog_status::ProgramStatus; item_count),
            };
            Ok(PyHmiHandleGeneric {
                inner: gen_handle,
                target: self.entry.var_slice_range.0,
                count: self.entry.var_slice_range.1,
                caster,
            })
        }
    }

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        use crate::hmi::proto::asg::*;
        parent_module.add_class::<PyAsgVarInterface>()?;
        parent_module.add_class::<AsgValue>()?;

        parent_module.add_class::<position_struct::PositionData>()?;
        parent_module.add_class::<position_struct::CartesianData>()?;
        parent_module.add_class::<position_struct::JointData>()?;
        parent_module.add_class::<position_struct::FrameData>()?;
        parent_module.add_class::<position_struct::FlipState>()?;
        parent_module.add_class::<position_struct::FrontBack>()?;
        parent_module.add_class::<position_struct::LeftRight>()?;
        parent_module.add_class::<position_struct::UpDown>()?;

        parent_module.add_class::<alarm_struct::AlarmData>()?;
        parent_module.add_class::<alarm_struct::AlarmSeverity>()?;
        parent_module.add_class::<AlarmSource>()?;
        parent_module.add_class::<alarm_struct::TimeData>()?;

        parent_module.add_class::<prog_status::ProgramStatus>()?;
        parent_module.add_class::<ProgramStatusKind>()?;
        parent_module.add_class::<prog_status::ProgramState>()?;

        parent_module.add_class::<BoolIoSignal>()?;
        parent_module.add_class::<IntIoSignal>()?;

        Ok(())
    }
}
