use std::borrow::Cow;

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
                if N > $cnt && !fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                let copy_len = N.min($cnt);
                paste::paste! {
                    for i in 0..copy_len {
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
                if N > $cnt && !fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                let copy_len = N.min($cnt);
                paste::paste! {
                    for i in 0..copy_len {
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
                if self.len() < N && !fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                let copy_len = N.min(self.len());
                paste::paste! {
                    for i in 0..copy_len {
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
                if self.len() < N && !fill_missing_nan {
                    return Err(JointDataSizeError(self.len()));
                }
                let mut arr: [D; N] = [D::nan(); N];
                let copy_len = N.min(self.len());
                paste::paste! {
                    for i in 0..copy_len {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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
#[derive(Debug, Clone)]
pub struct JointTemplate {
    pub axis: Cow<'static, [JointType]>,
}

use JointType::*;
use cfg_mixin::cfg_mixin;
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointTemplate {
    #[on(new)]
    pub fn new(axis: Vec<JointType>) -> Self {
        Self {
            axis: Cow::Owned(axis),
        }
    }

    #[cfg(off)]
    pub const fn new_const(axis: &'static [JointType]) -> Self {
        Self {
            axis: Cow::Borrowed(axis),
        }
    }

    #[on(classattr)]
    pub const SIX: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary, Rotary, Rotary]),
    };
    #[on(classattr)]
    pub const SIX_LINEAR_TRACK: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary, Rotary, Rotary, Linear]),
    };
    #[on(classattr)]
    pub const FOUR: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary]),
    };
    #[on(classattr)]
    pub const FOUR_LINEAR_TRACK: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary, Linear]),
    };
    #[on(classattr)]
    pub const FIVE: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary, Rotary]),
    };
    #[on(classattr)]
    pub const FIVE_LINEAR_TRACK: Self = Self {
        axis: Cow::Borrowed(&[Rotary, Rotary, Rotary, Rotary, Rotary, Linear]),
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
        template: &JointTemplate,
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
        Ok(self.convert_from(format, &template, joints))
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

#[cfg(test)]
mod tests {
    use super::*;

    const EPS_F32: f32 = 1e-4;
    const EPS_F64: f64 = 1e-9;

    fn approx_eq_f32(a: f32, b: f32) -> bool {
        (a - b).abs() <= EPS_F32 || ((a - b) / a.abs().max(b.abs()).max(1.0)).abs() <= EPS_F32
    }

    fn approx_eq_f64(a: f64, b: f64) -> bool {
        (a - b).abs() <= EPS_F64 || ((a - b) / a.abs().max(b.abs()).max(1.0)).abs() <= EPS_F64
    }

    fn assert_arr_close_f32(actual: [f32; 9], expected: [f32; 9], ctx: &str) {
        for i in 0..9 {
            assert!(
                approx_eq_f32(actual[i], expected[i]),
                "{ctx}: index {i} mismatch — got {} expected {}",
                actual[i],
                expected[i]
            );
        }
    }

    const ALL_FORMATS: [JointFormat; 4] = [
        JointFormat::AbsRad,
        JointFormat::FanucRad,
        JointFormat::AbsDeg,
        JointFormat::FanucDeg,
    ];

    fn all_templates() -> [(JointTemplate, &'static str); 6] {
        [
            (JointTemplate::SIX, "SIX"),
            (JointTemplate::SIX_LINEAR_TRACK, "SIX_LINEAR_TRACK"),
            (JointTemplate::FOUR, "FOUR"),
            (JointTemplate::FOUR_LINEAR_TRACK, "FOUR_LINEAR_TRACK"),
            (JointTemplate::FIVE, "FIVE"),
            (JointTemplate::FIVE_LINEAR_TRACK, "FIVE_LINEAR_TRACK"),
        ]
    }

    #[test]
    fn convert_from_identity_is_noop() {
        // Same format on both sides should always be a no-op regardless of template.
        let original: [f64; 9] = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0];
        for fmt in ALL_FORMATS {
            for (tpl, tpl_name) in all_templates() {
                let out = fmt.convert_from(fmt, &tpl, original);
                for i in 0..9 {
                    assert!(
                        approx_eq_f64(out[i], original[i]),
                        "identity convert {fmt}/{tpl_name} mutated index {i}: {} -> {}",
                        original[i],
                        out[i]
                    );
                }
            }
        }
    }

    #[test]
    fn convert_from_round_trips_across_all_format_pairs() {
        // For every (src, mid) pair, src -> mid -> src must reproduce the input.
        // This exercises every arm of the convert_from match table.
        let original: [f64; 9] = [-90.0, 30.0, -45.0, 180.0, -90.0, 360.0, 1.5, 0.0, 0.0];
        for src in ALL_FORMATS {
            for mid in ALL_FORMATS {
                for (tpl, tpl_name) in all_templates() {
                    let intermediate = mid.convert_from(src, &tpl, original);
                    let back = src.convert_from(mid, &tpl, intermediate);
                    for i in 0..9 {
                        assert!(
                            approx_eq_f64(back[i], original[i]),
                            "round-trip {src}->{mid}->{src} on {tpl_name} mismatch at idx {i}: \
                             original={} after_round_trip={}",
                            original[i],
                            back[i]
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn fanuc_to_abs_adjusts_only_j3_with_j2() {
        // FanucDeg -> AbsDeg: j3_abs = j3_fanuc + j2. Other axes untouched.
        let fanuc: [f64; 9] = [10.0, 25.0, -15.0, 40.0, 50.0, 60.0, 7.0, 8.0, 9.0];
        let abs =
            JointFormat::AbsDeg.convert_from(JointFormat::FanucDeg, &JointTemplate::SIX, fanuc);
        assert!(approx_eq_f64(abs[0], 10.0));
        assert!(approx_eq_f64(abs[1], 25.0));
        assert!(approx_eq_f64(abs[2], -15.0 + 25.0)); // j3 + j2
        for i in 3..9 {
            assert!(approx_eq_f64(abs[i], fanuc[i]), "axis {i} should be untouched");
        }
    }

    #[test]
    fn abs_to_fanuc_subtracts_j2_from_j3() {
        let abs: [f64; 9] = [10.0, 25.0, 10.0, 40.0, 50.0, 60.0, 0.0, 0.0, 0.0];
        let fanuc =
            JointFormat::FanucDeg.convert_from(JointFormat::AbsDeg, &JointTemplate::SIX, abs);
        assert!(approx_eq_f64(fanuc[0], 10.0));
        assert!(approx_eq_f64(fanuc[1], 25.0));
        assert!(approx_eq_f64(fanuc[2], 10.0 - 25.0));
        for i in 3..9 {
            assert!(approx_eq_f64(fanuc[i], abs[i]));
        }
    }

    #[test]
    fn deg_to_rad_only_touches_rotary_axes() {
        // SIX_LINEAR_TRACK has 6 rotary + 1 linear (index 6). The linear axis
        // value (a track position in mm) must NOT be deg/rad-converted.
        let deg: [f64; 9] = [180.0, 90.0, 90.0, 0.0, 0.0, 0.0, 1234.5, 0.0, 0.0];
        let rad = JointFormat::AbsRad.convert_from(
            JointFormat::AbsDeg,
            &JointTemplate::SIX_LINEAR_TRACK,
            deg,
        );
        assert!(approx_eq_f64(rad[0], std::f64::consts::PI));
        assert!(approx_eq_f64(rad[1], std::f64::consts::FRAC_PI_2));
        assert!(approx_eq_f64(rad[2], std::f64::consts::FRAC_PI_2));
        for i in 3..6 {
            assert!(approx_eq_f64(rad[i], 0.0));
        }
        // Linear track must not have been multiplied by π/180.
        assert!(
            approx_eq_f64(rad[6], 1234.5),
            "linear track axis was deg-converted: {}",
            rad[6]
        );
        // Beyond the template length, values pass through.
        assert!(approx_eq_f64(rad[7], 0.0));
        assert!(approx_eq_f64(rad[8], 0.0));
    }

    #[test]
    fn rad_to_deg_only_touches_rotary_axes() {
        let rad: [f64; 9] = [
            std::f64::consts::PI,
            std::f64::consts::FRAC_PI_2,
            -std::f64::consts::FRAC_PI_4,
            0.0,
            0.0,
            0.0,
            999.0, // linear track in mm
            0.0,
            0.0,
        ];
        let deg = JointFormat::AbsDeg.convert_from(
            JointFormat::AbsRad,
            &JointTemplate::SIX_LINEAR_TRACK,
            rad,
        );
        assert!(approx_eq_f64(deg[0], 180.0));
        assert!(approx_eq_f64(deg[1], 90.0));
        assert!(approx_eq_f64(deg[2], -45.0));
        assert!(approx_eq_f64(deg[6], 999.0), "linear track passed through");
    }

    #[test]
    fn convert_from_works_on_f32_arrays() {
        // The stmo and hspo packets carry [f32; 9]. Ensure conversion works at
        // the f32 precision used on the wire.
        let fanuc_deg: [f32; 9] = [-90.0, 30.0, -15.0, 180.0, -90.0, 0.0, 0.0, 0.0, 0.0];
        let abs_rad = JointFormat::AbsRad.convert_from(
            JointFormat::FanucDeg,
            &JointTemplate::SIX,
            fanuc_deg,
        );
        let back = JointFormat::FanucDeg.convert_from(
            JointFormat::AbsRad,
            &JointTemplate::SIX,
            abs_rad,
        );
        assert_arr_close_f32(back, fanuc_deg, "f32 fanuc_deg <-> abs_rad round-trip");
    }

    #[test]
    fn convert_from_works_on_vec() {
        // Vec<f64> is a separate JointRepr impl; confirm it produces identical
        // results to the equivalent [f64; 9] flow.
        let fanuc_deg_arr: [f64; 9] = [-90.0, 30.0, -15.0, 180.0, -90.0, 0.0, 0.0, 0.0, 0.0];
        let fanuc_deg_vec: Vec<f64> = fanuc_deg_arr.to_vec();
        let abs_rad_arr = JointFormat::AbsRad.convert_from(
            JointFormat::FanucDeg,
            &JointTemplate::SIX,
            fanuc_deg_arr,
        );
        let abs_rad_vec = JointFormat::AbsRad.convert_from(
            JointFormat::FanucDeg,
            &JointTemplate::SIX,
            fanuc_deg_vec,
        );
        for i in 0..9 {
            assert!(
                approx_eq_f64(abs_rad_arr[i], abs_rad_vec[i]),
                "Vec vs array conversion diverged at idx {i}: arr={} vec={}",
                abs_rad_arr[i],
                abs_rad_vec[i]
            );
        }
    }

    #[test]
    fn convert_from_does_not_use_extra_axes_beyond_template() {
        // FOUR has 4 rotary axes — the j5 / j6 / j7+ entries should pass through
        // any conversion untouched even though the array carries 9 slots.
        let abs_deg: [f64; 9] = [10.0, 20.0, 30.0, 40.0, 1234.5, 5678.0, 9999.0, 0.0, 0.0];
        let abs_rad =
            JointFormat::AbsRad.convert_from(JointFormat::AbsDeg, &JointTemplate::FOUR, abs_deg);
        assert!(approx_eq_f64(abs_rad[4], 1234.5));
        assert!(approx_eq_f64(abs_rad[5], 5678.0));
        assert!(approx_eq_f64(abs_rad[6], 9999.0));
    }

    #[test]
    fn template_display_distinguishes_linear_and_rotary() {
        let s = format!("{}", JointTemplate::SIX_LINEAR_TRACK);
        assert!(s.contains("R, R, R, R, R, R, L"), "got: {s}");
        let s = format!("{}", JointTemplate::FOUR);
        assert!(s.contains("R, R, R, R"), "got: {s}");
    }

    #[test]
    fn format_display_matches_variant_names() {
        assert_eq!(format!("{}", JointFormat::AbsRad), "AbsRad");
        assert_eq!(format!("{}", JointFormat::FanucRad), "FanucRad");
        assert_eq!(format!("{}", JointFormat::AbsDeg), "AbsDeg");
        assert_eq!(format!("{}", JointFormat::FanucDeg), "FanucDeg");
    }

    #[test]
    fn to_array_undersized_array_returns_error_when_no_fill() {
        // [f64; 4] -> [f64; 6] with fill_missing_nan=false must error,
        // never panic on out-of-bounds index.
        let src: [f64; 4] = [1.0, 2.0, 3.0, 4.0];
        let res: Result<[f64; 6], _> = src.to_array(false);
        assert!(res.is_err(), "expected JointDataSizeError, got {res:?}");
    }

    #[test]
    fn to_array_undersized_array_pads_with_nan_when_fill_requested() {
        let src: [f64; 4] = [1.0, 2.0, 3.0, 4.0];
        let res: [f64; 6] = src.to_array(true).expect("fill_missing_nan path must succeed");
        assert_eq!(res[0], 1.0);
        assert_eq!(res[1], 2.0);
        assert_eq!(res[2], 3.0);
        assert_eq!(res[3], 4.0);
        assert!(res[4].is_nan(), "tail should be NaN, got {}", res[4]);
        assert!(res[5].is_nan(), "tail should be NaN, got {}", res[5]);
    }

    #[test]
    fn to_array_oversized_array_truncates_to_n() {
        // [f64; 6] -> [f64; 4] keeps the first 4; the rest of the source
        // array is ignored regardless of fill flag.
        let src: [f64; 6] = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let res: [f64; 4] = src.to_array(false).unwrap();
        assert_eq!(res, [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn to_array_undersized_vec_returns_error_when_no_fill() {
        let src: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0];
        let res: Result<[f64; 6], _> = src.to_array(false);
        assert!(res.is_err(), "expected JointDataSizeError, got {res:?}");
    }

    #[test]
    fn to_array_undersized_vec_pads_with_nan_when_fill_requested() {
        let src: Vec<f64> = vec![1.0, 2.0, 3.0, 4.0];
        let res: [f64; 6] = src.to_array(true).unwrap();
        assert_eq!(res[0], 1.0);
        assert!(res[5].is_nan());
    }
}
