/// A trait to be implemented on a scalar value that can be used to represent a joint value in a variety of datastructures.
pub trait JointValue: Copy {
    fn to_f64(self) -> f64;
    fn from_f64(v: f64) -> Self;
    fn to_f32(self) -> f32;
    fn from_f32(v: f32) -> Self;
    fn nan() -> Self;
    fn zero() -> Self;
}

impl JointValue for f64 {
    fn to_f64(self) -> f64 {
        self
    }
    fn from_f64(v: f64) -> Self {
        v
    }
    fn to_f32(self) -> f32 {
        self as f32
    }
    fn from_f32(v: f32) -> Self {
        v as f64
    }
    fn nan() -> Self {
        f64::NAN
    }
    fn zero() -> Self {
        0.0
    }
}

impl JointValue for f32 {
    fn to_f64(self) -> f64 {
        self as f64
    }
    fn from_f64(v: f64) -> Self {
        v as f32
    }
    fn to_f32(self) -> f32 {
        self
    }
    fn from_f32(v: f32) -> Self {
        v
    }
    fn nan() -> Self {
        f32::NAN
    }
    fn zero() -> Self {
        0.0
    }
}

/// An enum represnting the unit quantity for a joint and how to handle conversions between different formats.
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JointType {
    Linear,
    Rotary,
}

impl std::fmt::Display for JointType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            JointType::Linear => "Linear",
            JointType::Rotary => "Rotary",
        };
        write!(f, "{}", s)
    }
}

/// A trait representing a joint representation that can be converted between different formats and units.
pub trait JointRepr: std::fmt::Debug {
    fn to_deg(self, mask: &[JointType]) -> Self;
    fn to_rad(self, mask: &[JointType]) -> Self;
    fn to_fanuc(self) -> Self;
    fn to_abs(self) -> Self;
    fn len(&self) -> usize;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
    fn to_array<const N: usize, D: JointValue>(
        self,
        fill_missing_nan: bool,
    ) -> Result<[D; N], JointDataSizeError>;
}

macro_rules! float_slice_joint {
    (&mut $float:ty, $cnt:literal, $pi:expr) => {
        impl JointRepr for &mut [$float; $cnt] {
            fn to_deg(self, mask: &[JointType]) -> Self {
                let limit = $cnt.min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] * 180.0) / $pi;
                    }
                }
                self
            }
            fn to_rad(self, mask: &[JointType]) -> Self {
                let limit = $cnt.min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] / 180.0) * $pi;
                    }
                }
                self
            }
            fn to_fanuc(self) -> Self {
                self[2] -= self[1];
                self
            }
            fn to_abs(self) -> Self {
                self[2] += self[1];
                self
            }

            fn len(&self) -> usize {
                self.iter().len()
            }

            fn to_array<const N: usize, D: JointValue>(
                self,
                fill_missing_nan: bool,
            ) -> Result<[D; N], JointDataSizeError> {
                if N > $cnt && fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                paste::paste! {
                    for i in 0..N {
                        arr[i] = D::[<from_ $float>](self[i]);
                    }
                }
                Ok(arr)
            }
        }
    };
    ($float:ty, $cnt:literal, $pi:expr) => {
        impl JointRepr for [$float; $cnt] {
            fn to_deg(mut self, mask: &[JointType]) -> Self {
                let limit = $cnt.min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] * 180.0) / $pi;
                    }
                }
                self
            }
            fn to_rad(mut self, mask: &[JointType]) -> Self {
                let limit = $cnt.min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] / 180.0) * $pi;
                    }
                }
                self
            }
            fn to_fanuc(mut self) -> Self {
                self[2] -= self[1];
                self
            }
            fn to_abs(mut self) -> Self {
                self[2] += self[1];
                self
            }
            fn len(&self) -> usize {
                <[$float]>::len(self)
            }
            fn to_array<const N: usize, D: JointValue>(
                self,
                fill_missing_nan: bool,
            ) -> Result<[D; N], JointDataSizeError> {
                if N > $cnt && fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                paste::paste! {
                    for i in 0..N {
                        arr[i] = D::[<from_ $float>](self[i]);
                    }
                }
                Ok(arr)
            }
        }
    };
}

float_slice_joint!(f64, 3, std::f64::consts::PI);
float_slice_joint!(&mut f64, 3, std::f64::consts::PI);
float_slice_joint!(f32, 3, std::f32::consts::PI);
float_slice_joint!(&mut f32, 3, std::f32::consts::PI);
float_slice_joint!(f64, 4, std::f64::consts::PI);
float_slice_joint!(&mut f64, 4, std::f64::consts::PI);
float_slice_joint!(f32, 4, std::f32::consts::PI);
float_slice_joint!(&mut f32, 4, std::f32::consts::PI);
float_slice_joint!(f64, 5, std::f64::consts::PI);
float_slice_joint!(&mut f64, 5, std::f64::consts::PI);
float_slice_joint!(f32, 5, std::f32::consts::PI);
float_slice_joint!(&mut f32, 5, std::f32::consts::PI);
float_slice_joint!(f64, 6, std::f64::consts::PI);
float_slice_joint!(&mut f64, 6, std::f64::consts::PI);
float_slice_joint!(f32, 6, std::f32::consts::PI);
float_slice_joint!(&mut f32, 6, std::f32::consts::PI);
float_slice_joint!(f64, 7, std::f64::consts::PI);
float_slice_joint!(&mut f64, 7, std::f64::consts::PI);
float_slice_joint!(f32, 7, std::f32::consts::PI);
float_slice_joint!(&mut f32, 7, std::f32::consts::PI);
float_slice_joint!(f64, 8, std::f64::consts::PI);
float_slice_joint!(&mut f64, 8, std::f64::consts::PI);
float_slice_joint!(f32, 8, std::f32::consts::PI);
float_slice_joint!(&mut f32, 8, std::f32::consts::PI);
float_slice_joint!(f64, 9, std::f64::consts::PI);
float_slice_joint!(&mut f64, 9, std::f64::consts::PI);
float_slice_joint!(f32, 9, std::f32::consts::PI);
float_slice_joint!(&mut f32, 9, std::f32::consts::PI);

macro_rules! float_vec_joint {
    (&mut $float:ty, $pi:expr) => {
        impl JointRepr for &mut Vec<$float> {
            fn to_deg(self, mask: &[JointType]) -> Self {
                let limit = self.len().min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] * 180.0) / $pi;
                    }
                }
                self
            }
            fn to_rad(self, mask: &[JointType]) -> Self {
                let limit = self.len().min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] / 180.0) * $pi;
                    }
                }
                self
            }
            fn to_fanuc(self) -> Self {
                if self.len() >= 3 {
                    self[2] -= self[1];
                }
                self
            }
            fn to_abs(self) -> Self {
                if self.len() >= 3 {
                    self[2] += self[1];
                }
                self
            }
            fn len(&self) -> usize {
                <Vec<$float>>::len(self)
            }
            fn to_array<const N: usize, D: JointValue>(
                self,
                fill_missing_nan: bool,
            ) -> Result<[D; N], JointDataSizeError> {
                if self.len() < N && fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                paste::paste! {
                    for i in 0..N {
                        arr[i] = D::[<from_ $float>](self[i]);
                    }
                }
                Ok(arr)
            }
        }
    };
    ($float:ty, $pi:expr) => {
        impl JointRepr for Vec<$float> {
            fn to_deg(mut self, mask: &[JointType]) -> Self {
                let limit = self.len().min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] * 180.0) / $pi;
                    }
                }
                self
            }
            fn to_rad(mut self, mask: &[JointType]) -> Self {
                let limit = self.len().min(mask.len());
                for i in 0..limit {
                    if mask[i] == JointType::Rotary {
                        self[i] = (self[i] / 180.0) * $pi;
                    }
                }
                self
            }
            fn to_fanuc(mut self) -> Self {
                if self.len() >= 3 {
                    self[2] -= self[1];
                }
                self
            }
            fn to_abs(mut self) -> Self {
                if self.len() >= 3 {
                    self[2] += self[1];
                }
                self
            }
            fn len(&self) -> usize {
                <Vec<$float>>::len(self)
            }
            fn to_array<const N: usize, D: JointValue>(
                self,
                fill_missing_nan: bool,
            ) -> Result<[D; N], JointDataSizeError> {
                if self.len() < N && fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                paste::paste! {
                    for i in 0..N {
                        arr[i] = D::[<from_ $float>](self[i]);
                    }
                }
                Ok(arr)
            }
        }
    };
}

float_vec_joint!(f64, std::f64::consts::PI);
float_vec_joint!(&mut f64, std::f64::consts::PI);
float_vec_joint!(f32, std::f32::consts::PI);
float_vec_joint!(&mut f32, std::f32::consts::PI);

#[derive(Debug, Clone, Copy)]
pub struct JointDataSizeError(pub usize);
impl std::fmt::Display for JointDataSizeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Joint data len is too small: {}", self.0)
    }
}
impl std::error::Error for JointDataSizeError {}

/// A datastructure representing the units/[``JointType``] of each joint in a robot, which can be used to convert between different formats and units.
///
/// [JointTemplate] comes with a variety of pre-defined templates for common robot configurations,
/// but can also be constructed from a list of [JointType]s to represent custom robot configurations.
///
/// Premade templates include:
/// - [JointTemplate::SIX]: A 6-axis robot with all rotary joints.
/// - [JointTemplate::SIX_LINEAR_TRACK]: A 7-axis robot with 6 rotary joints and a linear track.
/// - [JointTemplate::FOUR]: A 4-axis robot with all rotary joints.
/// - [JointTemplate::FOUR_LINEAR_TRACK]: A 5-axis robot with 4 rotary joints and a linear track.
/// - [JointTemplate::FIVE]: A 5-axis robot with all rotary joints.
/// - [JointTemplate::FIVE_LINEAR_TRACK]: A 6-axis robot with 5 rotary joints and a linear track.
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy)]
pub struct JointTemplate {
    pub axis: &'static [JointType],
}

use JointType::*;
use cfg_mixin::cfg_mixin;
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointTemplate {
    #[on(new)]
    pub fn new(axis: Vec<JointType>) -> Self {
        Self {
            axis: Box::leak(axis.into_boxed_slice()),
        }
    }

    #[cfg(off)]
    pub const fn new_const(axis: &'static [JointType]) -> Self {
        Self { axis }
    }

    #[on(classattr)]
    pub const SIX: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary, Rotary, Rotary],
    };
    #[on(classattr)]
    pub const SIX_LINEAR_TRACK: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary, Rotary, Rotary, Linear],
    };
    #[on(classattr)]
    pub const FOUR: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary],
    };
    #[on(classattr)]
    pub const FOUR_LINEAR_TRACK: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary, Linear],
    };
    #[on(classattr)]
    pub const FIVE: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary, Rotary],
    };
    #[on(classattr)]
    pub const FIVE_LINEAR_TRACK: Self = Self {
        axis: &[Rotary, Rotary, Rotary, Rotary, Rotary, Linear],
    };
}

impl std::fmt::Display for JointTemplate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s: Vec<&str> = self
            .axis
            .iter()
            .map(|t| match t {
                JointType::Linear => "L",
                JointType::Rotary => "R",
            })
            .collect();
        write!(f, "JointTemplate[{}]", s.join(", "))
    }
}

/// An enum representing the format of joint data,
/// which can be used to convert between different formats and units to determine the appropriate conversions for each joint.
///
/// This enum is often passed into the API's in this library to know how to return the information in the appropriate format
/// however users have access to [JointFormat::convert_from] to convert between formats on their own data as well.
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object))]
#[derive(Debug, Clone, Copy)]
pub enum JointFormat {
    AbsRad,
    FanucRad,
    AbsDeg,
    FanucDeg,
}

impl JointFormat {
    pub fn convert_from<T: JointRepr>(
        &self,
        format: JointFormat,
        template: JointTemplate,
        joints: T,
    ) -> T {
        let mask = &template.axis;
        match self {
            JointFormat::FanucRad => match format {
                JointFormat::FanucRad => joints,
                JointFormat::AbsRad => joints.to_fanuc(),
                JointFormat::FanucDeg => joints.to_rad(mask),
                JointFormat::AbsDeg => joints.to_fanuc().to_rad(mask),
            },
            JointFormat::AbsRad => match format {
                JointFormat::FanucRad => joints.to_abs(),
                JointFormat::AbsRad => joints,
                JointFormat::FanucDeg => joints.to_abs().to_rad(mask),
                JointFormat::AbsDeg => joints.to_rad(mask),
            },
            JointFormat::FanucDeg => match format {
                JointFormat::FanucRad => joints.to_deg(mask),
                JointFormat::AbsRad => joints.to_fanuc().to_deg(mask),
                JointFormat::FanucDeg => joints,
                JointFormat::AbsDeg => joints.to_fanuc(),
            },
            JointFormat::AbsDeg => match format {
                JointFormat::FanucRad => joints.to_abs().to_deg(mask),
                JointFormat::AbsRad => joints.to_deg(mask),
                JointFormat::FanucDeg => joints.to_abs(),
                JointFormat::AbsDeg => joints,
            },
        }
    }
}

#[cfg(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointFormat {
    #[pyo3(name = "convert_from")]
    pub fn py_convert_from(
        &self,
        format: JointFormat,
        template: JointTemplate,
        joints: pyo3::Bound<pyo3::types::PySequence>,
    ) -> pyo3::PyResult<Vec<f64>> {
        use pyo3::types::PyAnyMethods;

        let joints: Vec<f64> = joints
            .try_iter()?
            .map(|item| item.and_then(|obj| obj.extract::<f64>()))
            .collect::<pyo3::PyResult<Vec<f64>>>()?;
        if joints.len() < template.axis.len() {
            return Err(pyo3::exceptions::PyValueError::new_err(format!(
                "joints list must have at least {} elements",
                template.axis.len()
            )));
        }
        Ok(self.convert_from(format, template, joints))
    }
}

impl std::fmt::Display for JointFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            JointFormat::AbsRad => "AbsRad",
            JointFormat::FanucRad => "FanucRad",
            JointFormat::AbsDeg => "AbsDeg",
            JointFormat::FanucDeg => "FanucDeg",
        };
        write!(f, "{}", s)
    }
}
