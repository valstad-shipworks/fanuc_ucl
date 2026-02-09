use super::asg_handle::{
    self, AlarmArgs, AlarmSource, BoolIoArgs, BoolIoSignal, IntIoArgs, IntIoSignal, NumRegArgs,
    PosRegArgs, ProgramStatusArgs, ProgramStatusKind, StringRegArgs, SysVarArgs,
};
use super::hmi_handle::py::{
    PyHmiHandleGeneric, py_caster_array, py_caster_null, py_caster_singular,
};
use super::*;
use crate::hmi::asg_handle::CurPosArgs;
use crate::hmi::asg_handle::py::PyAsgVarInterface;
use crate::hmi::proto::asg::{alarm_struct, position_struct, prog_status};
use crate::hmi::proto::ports::DataPort;
use pyo3::exceptions::{PyKeyError, PyTypeError};
use pyo3::types::{PyBool, PyDict, PyFloat, PyInt, PyString, PyTuple};
use pyo3::{PyTypeInfo, prelude::*};
use std::marker::PhantomData;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[pyclass(name = "_DataPortVariants", from_py_object)]
pub enum DataPortVariants {
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
    GroupInput,
    GroupOutput,
    AnalogInput,
    AnalogOutput,
    Register,
    Command,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[pyclass(name = "DataPort", from_py_object)]
pub struct DataPortGeneric {
    variant: DataPortVariants,
    can_read: bool,
    can_write: bool,
}

fn kwargs_dict<'py>(py: Python<'py>, kwargs: Option<&Bound<'py, PyDict>>) -> Bound<'py, PyDict> {
    match kwargs {
        Some(dict) => dict.clone(),
        None => PyDict::new(py),
    }
}

fn require_kwarg<'py, T>(kwargs: &Bound<'py, PyDict>, name: &str) -> PyResult<T>
where
    for<'a> T: pyo3::FromPyObject<'a, 'py>,
    for<'a> <T as pyo3::FromPyObject<'a, 'py>>::Error: Into<PyErr>
{
    kwargs
        .get_item(name)?
        .ok_or_else(|| PyKeyError::new_err(format!("Missing required keyword argument '{name}'")))?
        .extract::<T>()
        .map_err(Into::into)
}

fn optional_kwarg<'py, T>(kwargs: &Bound<'py, PyDict>, name: &str) -> PyResult<Option<T>>
where
    T: FromPyObjectOwned<'py, Error = PyErr>,
{
    match kwargs.get_item(name)? {
        Some(value) if value.is_none() => Ok(None),
        Some(value) => value.extract::<T>().map(Some),
        None => Ok(None),
    }
}

fn extract_range_kwarg<'py>(kwargs: &Bound<'py, PyDict>) -> PyResult<Option<(u16, u16)>> {
    optional_kwarg::<(u16, u16)>(kwargs, "range")
}


fn make_asg_interface<T: AsgArgument>(driver: &mut HmiDriver, arg: T, count: usize, timeout: Duration) -> DriverResult<PyAsgVarInterface> {
    let mut entry = arg.to_asg_entry();
    tracing::trace!("Making ASG interface for entry: {}", entry);
    entry.size *= count as u16;
    if driver.asg_entries.is_empty() {
        entry.address = 1;
        let entry_arc = Arc::new(entry);
        driver.send_asg_cmd(entry_arc.clone(), timeout)?;
        driver.asg_entries
            .insert(entry_arc.var_name.clone(), entry_arc.clone());
        return Ok(PyAsgVarInterface::new(entry_arc, count, T::WRITABLE));
    }
    if let Some(entry) = driver.asg_entries.get(&entry.var_name) {
        return Ok(PyAsgVarInterface::new(entry.clone(), count, T::WRITABLE));
    }
    let mut max_address = 0u16;
    for (_, existing_entry) in driver.asg_entries.iter() {
        let end_address = existing_entry.address + existing_entry.size;
        if end_address > max_address {
            max_address = end_address;
        }
    }
    entry.address = max_address;
    let entry_arc = Arc::new(entry);
    driver.send_asg_cmd(entry_arc.clone(), timeout)?;
    driver.asg_entries
        .insert(entry_arc.var_name.clone(), entry_arc.clone());
    Ok(PyAsgVarInterface::new(entry_arc, count, T::WRITABLE))
}

fn is_sequence(py: Python<'_>, value: &Bound<PyAny>) -> DriverResult<bool> {
    let typing = py.import("typing")?;
    let sequence_type = typing.getattr("Sequence")?;
    let string_type = py.get_type::<pyo3::types::PyString>();
    let is_seq = value.is_instance(&sequence_type)?;
    let is_str = value.is_instance(&string_type)?;
    Ok(is_seq && !is_str)
}

fn checked_extract_array<T: DataPort>(
    py: Python<'_>,
    value: &Bound<PyAny>,
) -> DriverResult<Box<[T::ValueType]>> {
    if is_sequence(py, value)? {
        let typ = T::PyType::type_object(py);
        let mut out: Vec<T::ValueType> = Vec::new();
        for item in value.try_iter()? {
            let item = item?;
            if item.is_instance(&typ)? {
                let v = item.extract::<T::ValueType>()?;
                out.push(v);
            } else {
                return Err(pyo3::exceptions::PyTypeError::new_err(format!(
                    "Expected sequence of type {}",
                    typ.name()?
                )));
            }
        }
        Ok(out.into_boxed_slice())
    } else {
        Err(pyo3::exceptions::PyTypeError::new_err(
            "Expected a sequence type",
        ))
    }
}

fn checked_extract_single<T: DataPort>(
    py: Python<'_>,
    value: &Bound<PyAny>,
) -> DriverResult<T::ValueType> {
    let typ = T::PyType::type_object(py);
    if value.is_instance(&typ)? {
        let v = value.extract::<T::ValueType>()?;
        Ok(v)
    } else {
        Err(pyo3::exceptions::PyTypeError::new_err(format!(
            "Expected type {}",
            typ.name()?
        )))
    }
}

const DEFAULT_ASG_TIMEOUT_SECS: f64 = 0.016; // 16ms


#[pymethods]
impl HmiDriver {
    #[new]
    #[pyo3(signature = (addr))]
    pub fn py_new(addr: Bound<PyAny>) -> DriverResult<Self> {
        let ip = addr.extract::<IpAddr>()?;
        Ok(HmiDriver::new(ip))
    }

    #[pyo3(
            signature = (timeout_secs = DEFAULT_CONNECT_TIMEOUT_SECS, thread_config=None),
            name = "connect"
        )]
    pub fn py_connect(
        &mut self,
        timeout_secs: f64,
        thread_config: Option<ThreadConfig>,
    ) -> DriverResult<()> {
        if timeout_secs <= 0.0 {
            return Err(HmiError::Other("timeout_secs must be positive".into()).into());
        }
        let timeout = Duration::from_secs_f64(timeout_secs);
        HmiDriver::connect(self, Some(timeout), thread_config)
    }

    #[pyo3(name = "disconnect")]
    pub fn py_disconnect(&mut self) -> DriverResult<()> {
        HmiDriver::disconnect(self)
    }

    #[pyo3(name = "is_connected")]
    pub fn py_is_connected(&self) -> bool {
        HmiDriver::is_connected(self)
    }

    #[pyo3(name = "register_asg", signature = (*args, **kwargs))]
    pub fn register_asg_py<'py>(
        &mut self,
        py: Python<'py>,
        args: &Bound<'py, PyTuple>,
        kwargs: Option<&Bound<'py, PyDict>>,
    ) -> PyResult<PyAsgVarInterface> {
        if args.len() == 0 {
            return Err(PyTypeError::new_err(
                "register_asg(PositionData) expects atleast one positional type argument",
            ));
        }
        if args.len() > 2 {
            return Err(PyTypeError::new_err(
                "register_asg() expects at most two positional arguments",
            ));
        }
        if args.len() == 2 {
            let typ = args.get_item(0)?;
            if !typ.is(py.get_type::<position_struct::PositionData>()) {
                return Err(PyTypeError::new_err(
                    "Two positional arguments are only supported for PositionData type",
                ));
            }
        }
        let tag = args.get_item(0)?;
        let kwargs = kwargs_dict(py, kwargs);
        let timeout_secs = optional_kwarg::<f64>(&kwargs, "timeout_secs")?
            .unwrap_or(DEFAULT_ASG_TIMEOUT_SECS);
        let timeout = Duration::from_secs_f64(timeout_secs);
        if tag.is(py.get_type::<position_struct::PositionData>()) {
            let current = args.get_item(1)
                .and_then(|item| item.extract::<bool>())
                .map_err(|_| PyTypeError::new_err("Second positional argument must be a boolean indicating whether to use current position"))?;
            if current {
                let frame = require_kwarg::<i8>(&kwargs, "frame")?;
                let group = optional_kwarg::<u8>(&kwargs, "group")?;
                let range = extract_range_kwarg(&kwargs)?;
                let arg = CurPosArgs {
                    frame,
                    group,
                    range,
                };
                return Ok(make_asg_interface(self, arg, 1, timeout)?);
            } else {
                let index = require_kwarg::<u16>(&kwargs, "index")?;
                let group = optional_kwarg::<u8>(&kwargs, "group")?;
                let range = extract_range_kwarg(&kwargs)?;
                let arg = PosRegArgs {
                    index,
                    group,
                    range,
                };
                return Ok(make_asg_interface(self, arg, 1, timeout)?);
            }
        }
        if tag.get_type().is(py.get_type::<BoolIoSignal>()) {
            let signal = tag.extract::<BoolIoSignal>()?;
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let simulation = require_kwarg::<bool>(&kwargs, "simulation")?;
            let arg = BoolIoArgs {
                signal,
                index,
                simulation,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        if tag.get_type().is(py.get_type::<IntIoSignal>()) {
            let signal = tag.extract::<IntIoSignal>()?;
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let simulation = require_kwarg::<bool>(&kwargs, "simulation")?;
            let arg = IntIoArgs {
                signal,
                index,
                simulation,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        if tag.is(py.get_type::<alarm_struct::AlarmData>()) {
            let source = require_kwarg::<AlarmSource>(&kwargs, "source")?;
            let line = require_kwarg::<u16>(&kwargs, "line")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = AlarmArgs {
                source,
                line,
                range,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        if tag.is(py.get_type::<prog_status::ProgramStatus>()) {
            let task = require_kwarg::<u16>(&kwargs, "task")?;
            let kind = require_kwarg::<ProgramStatusKind>(&kwargs, "kind")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = ProgramStatusArgs { task, kind, range };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        if tag.is(py.get_type::<PyString>()) {
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = StringRegArgs { index, range };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        if tag.is(py.get_type::<PyFloat>()) {
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = NumRegArgs { index, range };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        Err(PyTypeError::new_err(
            "Unsupported ASG type for register_asg",
        ))
    }

    #[pyo3(name = "register_sysvar_asg", signature = (*args, **kwargs))]
    pub fn register_sysvar_asg_py<'py>(
        &mut self,
        py: Python<'py>,
        args: &Bound<'py, PyTuple>,
        kwargs: Option<&Bound<'py, PyDict>>,
    ) -> PyResult<PyAsgVarInterface> {
        if args.len() != 1 {
            return Err(PyTypeError::new_err(
                "register_sysvar_asg() expects exactly one positional type argument",
            ));
        }
        let tag = args.get_item(0)?;
        let kwargs = kwargs_dict(py, kwargs);
        let var_name = require_kwarg::<String>(&kwargs, "name")?;
        let range = extract_range_kwarg(&kwargs)?;
        let timeout_secs = optional_kwarg::<f64>(&kwargs, "timeout_secs")?
            .unwrap_or(DEFAULT_ASG_TIMEOUT_SECS);
        let timeout = Duration::from_secs_f64(timeout_secs);
        if tag.is(py.get_type::<PyBool>()) {
            let arg = SysVarArgs::<bool> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        } else if tag.is(py.get_type::<PyInt>()) {
            let arg = SysVarArgs::<i32> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        } else if tag.is(py.get_type::<PyFloat>()) {
            let arg = SysVarArgs::<f32> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        } else if tag.is(py.get_type::<PyString>()) {
            let arg = SysVarArgs::<String> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        } else if tag.is(py.get_type::<position_struct::PositionData>()) {
            let arg = SysVarArgs::<position_struct::PositionData> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, 1, timeout)?);
        }
        Err(PyTypeError::new_err(
            "Unsupported system variable type for register_sysvar_asg",
        ))
    }


    #[pyo3(name = "register_asg_array", signature = (*args, **kwargs))]
    pub fn register_asg_array_py<'py>(
        &mut self,
        py: Python<'py>,
        args: &Bound<'py, PyTuple>,
        kwargs: Option<&Bound<'py, PyDict>>,
    ) -> PyResult<PyAsgVarInterface> {
        if args.len() != 2 {
            return Err(PyTypeError::new_err(
                "register_asg_array() expects exactly two positional arguments",
            ));
        }
        let tag = args.get_item(0)?;
        let count = args.get_item(1)?.extract::<usize>()?;
        let kwargs = kwargs_dict(py, kwargs);
        let timeout_secs = optional_kwarg::<f64>(&kwargs, "timeout_secs")?
            .unwrap_or(DEFAULT_ASG_TIMEOUT_SECS);
        let timeout = Duration::from_secs_f64(timeout_secs);
        if tag.is(py.get_type::<position_struct::PositionData>()) {
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let group = optional_kwarg::<u8>(&kwargs, "group")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = PosRegArgs {
                index,
                group,
                range,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.get_type().is(py.get_type::<BoolIoSignal>()) {
            let signal = tag.extract::<BoolIoSignal>()?;
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let simulation = require_kwarg::<bool>(&kwargs, "simulation")?;
            let arg = BoolIoArgs {
                signal,
                index,
                simulation,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.get_type().is(py.get_type::<IntIoSignal>()) {
            let signal = tag.extract::<IntIoSignal>()?;
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let simulation = require_kwarg::<bool>(&kwargs, "simulation")?;
            let arg = IntIoArgs {
                signal,
                index,
                simulation,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.is(py.get_type::<alarm_struct::AlarmData>()) {
            let source = require_kwarg::<AlarmSource>(&kwargs, "source")?;
            let line = require_kwarg::<u16>(&kwargs, "line")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = AlarmArgs {
                source,
                line,
                range,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.is(py.get_type::<prog_status::ProgramStatus>()) {
            let task = require_kwarg::<u16>(&kwargs, "task")?;
            let kind = require_kwarg::<ProgramStatusKind>(&kwargs, "kind")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = ProgramStatusArgs { task, kind, range };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.is(py.get_type::<PyString>()) {
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = StringRegArgs { index, range };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        if tag.is(py.get_type::<PyFloat>()) {
            let index = require_kwarg::<u16>(&kwargs, "index")?;
            let range = extract_range_kwarg(&kwargs)?;
            let arg = NumRegArgs { index, range };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        Err(PyTypeError::new_err(
            "Unsupported ASG type for register_asg",
        ))
    }

    #[pyo3(name = "register_sysvar_asg_array", signature = (*args, **kwargs))]
    pub fn register_sysvar_asg_array_py<'py>(
        &mut self,
        py: Python<'py>,
        args: &Bound<'py, PyTuple>,
        kwargs: Option<&Bound<'py, PyDict>>,
    ) -> PyResult<PyAsgVarInterface> {
        if args.len() != 2 {
            return Err(PyTypeError::new_err(
                "register_sysvar_asg_array() expects exactly two positional arguments",
            ));
        }
        let tag = args.get_item(0)?;
        let count = args.get_item(1)?.extract::<usize>()?;
        let kwargs = kwargs_dict(py, kwargs);
        let var_name = require_kwarg::<String>(&kwargs, "name")?;
        let range = extract_range_kwarg(&kwargs)?;
        let timeout_secs = optional_kwarg::<f64>(&kwargs, "timeout_secs")?
            .unwrap_or(DEFAULT_ASG_TIMEOUT_SECS);
        let timeout = Duration::from_secs_f64(timeout_secs);
        if tag.is(py.get_type::<PyBool>()) {
            let arg = SysVarArgs::<bool> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        } else if tag.is(py.get_type::<PyInt>()) {
            let arg = SysVarArgs::<i32> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        } else if tag.is(py.get_type::<PyFloat>()) {
            let arg = SysVarArgs::<f32> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        } else if tag.is(py.get_type::<PyString>()) {
            let arg = SysVarArgs::<String> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        } else if tag.is(py.get_type::<position_struct::PositionData>()) {
            let arg = SysVarArgs::<position_struct::PositionData> {
                var_name,
                range,
                _marker: PhantomData,
            };
            return Ok(make_asg_interface(self, arg, count, timeout)?);
        }
        Err(PyTypeError::new_err(
            "Unsupported system variable type for register_sysvar_asg_array",
        ))
    }

    #[pyo3(signature = (port, index, value), name = "write")]
    pub fn write_py(
        &self,
        py: Python<'_>,
        port: DataPortGeneric,
        index: usize,
        value: Bound<PyAny>,
    ) -> DriverResult<PyHmiHandleGeneric> {
        if !port.can_write {
            return Err(pyo3::exceptions::PyPermissionError::new_err(
                "Port is not writable",
            ));
        }
        match port.variant {
            DataPortVariants::DigitalOutput => {
                self.write_variant::<proto::ports::DigitalOutput>(py, index, &value)
            }
            DataPortVariants::RobotOutput => {
                self.write_variant::<proto::ports::RobotOutput>(py, index, &value)
            }
            DataPortVariants::UopOutput => {
                self.write_variant::<proto::ports::UopOutput>(py, index, &value)
            }
            DataPortVariants::SopOutput => {
                self.write_variant::<proto::ports::SopOutput>(py, index, &value)
            }
            DataPortVariants::WeldOutput => {
                self.write_variant::<proto::ports::WeldOutput>(py, index, &value)
            }
            DataPortVariants::WireStickOutput => {
                self.write_variant::<proto::ports::WireStickOutput>(py, index, &value)
            }
            DataPortVariants::GroupOutput => {
                self.write_variant::<proto::ports::GroupOutput>(py, index, &value)
            }
            DataPortVariants::AnalogOutput => {
                self.write_variant::<proto::ports::AnalogOutput>(py, index, &value)
            }
            DataPortVariants::Register => {
                self.write_variant::<proto::ports::Register>(py, index, &value)
            }
            DataPortVariants::Command => {
                self.write_variant::<proto::ports::Command>(py, index, &value)
            }
            _ => Err(HmiError::Other("Unsupported port for write".to_string()).into()),
        }
    }

    #[pyo3(signature = (port, index, value), name = "write_unsafe")]
    pub fn write_unsafe_py(
        &self,
        py: Python<'_>,
        port: DataPortGeneric,
        index: usize,
        value: Bound<PyAny>,
    ) -> DriverResult<PyHmiHandleGeneric> {
        match port.variant {
            DataPortVariants::DigitalInput => {
                self.write_variant_unsafe::<proto::ports::DigitalInput>(py, index, &value)
            }
            DataPortVariants::DigitalOutput => {
                self.write_variant_unsafe::<proto::ports::DigitalOutput>(py, index, &value)
            }
            DataPortVariants::RobotInput => {
                self.write_variant_unsafe::<proto::ports::RobotInput>(py, index, &value)
            }
            DataPortVariants::RobotOutput => {
                self.write_variant_unsafe::<proto::ports::RobotOutput>(py, index, &value)
            }
            DataPortVariants::UopInput => {
                self.write_variant_unsafe::<proto::ports::UopInput>(py, index, &value)
            }
            DataPortVariants::UopOutput => {
                self.write_variant_unsafe::<proto::ports::UopOutput>(py, index, &value)
            }
            DataPortVariants::SopInput => {
                self.write_variant_unsafe::<proto::ports::SopInput>(py, index, &value)
            }
            DataPortVariants::SopOutput => {
                self.write_variant_unsafe::<proto::ports::SopOutput>(py, index, &value)
            }
            DataPortVariants::WeldInput => {
                self.write_variant_unsafe::<proto::ports::WeldInput>(py, index, &value)
            }
            DataPortVariants::WeldOutput => {
                self.write_variant_unsafe::<proto::ports::WeldOutput>(py, index, &value)
            }
            DataPortVariants::WireStickInput => {
                self.write_variant_unsafe::<proto::ports::WireStickInput>(py, index, &value)
            }
            DataPortVariants::WireStickOutput => {
                self.write_variant_unsafe::<proto::ports::WireStickOutput>(py, index, &value)
            }
            DataPortVariants::GroupInput => {
                self.write_variant_unsafe::<proto::ports::GroupInput>(py, index, &value)
            }
            DataPortVariants::GroupOutput => {
                self.write_variant_unsafe::<proto::ports::GroupOutput>(py, index, &value)
            }
            DataPortVariants::AnalogInput => {
                self.write_variant_unsafe::<proto::ports::AnalogInput>(py, index, &value)
            }
            DataPortVariants::AnalogOutput => {
                self.write_variant_unsafe::<proto::ports::AnalogOutput>(py, index, &value)
            }
            DataPortVariants::Register => {
                self.write_variant_unsafe::<proto::ports::Register>(py, index, &value)
            }
            DataPortVariants::Command => {
                self.write_variant_unsafe::<proto::ports::Command>(py, index, &value)
            }
        }
    }

    #[pyo3(signature = (port, index, count = 1), name = "read")]
    pub fn read_py<'a>(
        &self,
        py: Python<'a>,
        port: DataPortGeneric,
        index: usize,
        count: usize,
    ) -> DriverResult<PyHmiHandleGeneric> {
        if !port.can_read {
            return Err(pyo3::exceptions::PyPermissionError::new_err(
                "Port is not readable",
            ));
        }
        match port.variant {
            DataPortVariants::DigitalInput => {
                self.read_variant::<proto::ports::DigitalInput>(py, index, count)
            }
            DataPortVariants::DigitalOutput => {
                self.read_variant::<proto::ports::DigitalOutput>(py, index, count)
            }
            DataPortVariants::RobotInput => {
                self.read_variant::<proto::ports::RobotInput>(py, index, count)
            }
            DataPortVariants::RobotOutput => {
                self.read_variant::<proto::ports::RobotOutput>(py, index, count)
            }
            DataPortVariants::UopInput => {
                self.read_variant::<proto::ports::UopInput>(py, index, count)
            }
            DataPortVariants::UopOutput => {
                self.read_variant::<proto::ports::UopOutput>(py, index, count)
            }
            DataPortVariants::SopInput => {
                self.read_variant::<proto::ports::SopInput>(py, index, count)
            }
            DataPortVariants::SopOutput => {
                self.read_variant::<proto::ports::SopOutput>(py, index, count)
            }
            DataPortVariants::WeldInput => {
                self.read_variant::<proto::ports::WeldInput>(py, index, count)
            }
            DataPortVariants::WeldOutput => {
                self.read_variant::<proto::ports::WeldOutput>(py, index, count)
            }
            DataPortVariants::WireStickInput => {
                self.read_variant::<proto::ports::WireStickInput>(py, index, count)
            }
            DataPortVariants::WireStickOutput => {
                self.read_variant::<proto::ports::WireStickOutput>(py, index, count)
            }
            DataPortVariants::GroupInput => {
                self.read_variant::<proto::ports::GroupInput>(py, index, count)
            }
            DataPortVariants::GroupOutput => {
                self.read_variant::<proto::ports::GroupOutput>(py, index, count)
            }
            DataPortVariants::AnalogInput => {
                self.read_variant::<proto::ports::AnalogInput>(py, index, count)
            }
            DataPortVariants::AnalogOutput => {
                self.read_variant::<proto::ports::AnalogOutput>(py, index, count)
            }
            DataPortVariants::Register => {
                self.read_variant::<proto::ports::Register>(py, index, count)
            }
            _ => Err(HmiError::Other("Unsupported port for read".to_string()).into()),
        }
    }

    #[pyo3(signature = (), name = "clear_alarms")]
    pub fn clear_alarms_py(&self) -> PyResult<PyHmiHandleGeneric> {
        let handle = self.write::<ports::Command>(0, "CLRALM".to_string())?;
        Ok(PyHmiHandleGeneric {
            inner: handle.generic(),
            target: handle.target,
            count: handle.count,
            caster: py_caster_null::<()>,
        })
    }
}

impl HmiDriver {
    fn write_variant<T: WritableDataPort>(
        &self,
        py: Python<'_>,
        index: usize,
        value: &Bound<PyAny>,
    ) -> DriverResult<PyHmiHandleGeneric> {
        if is_sequence(py, value)? {
            let vals = checked_extract_array::<T>(py, value)?;
            let handle = self.write_array::<T>(index, &vals)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_null::<T>,
            })
        } else {
            let val = checked_extract_single::<T>(py, value)?;
            let handle = self.write::<T>(index, val)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_null::<T>,
            })
        }
    }

    fn write_variant_unsafe<T: UnsafelyWritableDataPort>(
        &self,
        py: Python<'_>,
        index: usize,
        value: &Bound<PyAny>,
    ) -> DriverResult<PyHmiHandleGeneric> {
        if is_sequence(py, value)? {
            let vals = checked_extract_array::<T>(py, value)?;
            let handle = self.write_array_unsafe::<T>(index, &vals)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_null::<T>,
            })
        } else {
            let val = checked_extract_single::<T>(py, value)?;
            let handle = self.write_unsafe::<T>(index, val)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_null::<T>,
            })
        }
    }

    fn read_variant<'a, T: ReadableDataPort>(
        &self,
        _py: Python<'a>,
        index: usize,
        count: usize,
    ) -> DriverResult<PyHmiHandleGeneric>
    where
        T::ValueType: Send + Sync + 'static,
    {
        if count == 0 {
            Err(pyo3::exceptions::PyValueError::new_err(
                "Count must be positive",
            ))
        } else if count > 1 {
            let handle = self.read_array::<T>(index, count)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_array::<T>,
            })
        } else {
            let handle = self.read::<T>(index)?;
            Ok(PyHmiHandleGeneric {
                inner: handle.generic(),
                target: handle.target,
                count: handle.count,
                caster: py_caster_singular::<T>,
            })
        }
    }
}

fn add_const_data_port_generic_to_module(
    module: &Bound<'_, PyModule>,
    variant: DataPortVariants,
    can_read: bool,
    can_write: bool,
) -> PyResult<()> {
    module.add(
        format!("{:?}", variant).as_str(),
        DataPortGeneric {
            variant,
            can_read,
            can_write,
        },
    )
}

pub fn register_child_module(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
    let child_module = PyModule::new(parent_module.py(), "hmi")?;
    child_module.add_class::<HmiDriver>()?;
    child_module.add_class::<DataPortVariants>()?;
    child_module.add_class::<DataPortGeneric>()?;

    hmi_handle::py::register(&child_module)?;
    asg_handle::py::register(&child_module)?;

    macro_rules! register_ports {
            ( $( $port_name:ident ),* $(,)? ) => {
                $(
                    add_const_data_port_generic_to_module(
                        &child_module,
                        DataPortVariants::$port_name,
                        ::impls::impls!(proto::ports::$port_name: proto::ports::ReadableDataPort),
                        ::impls::impls!(proto::ports::$port_name: proto::ports::WritableDataPort),
                    )?;
                )*
            };
        }

    register_ports! {
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
        GroupInput,
        GroupOutput,
        AnalogInput,
        AnalogOutput,
        Register,
        Command,
    }

    parent_module.add_submodule(&child_module)
}
