use cfg_mixin::cfg_mixin;
use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::joints::{JointRepr, JointType, JointValue, JointFormat, JointTemplate};

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum OnOff {
    ON,
    OFF,
}
pub type OnOnly = monostate::MustBe!("ON");
pub type OffOnly = monostate::MustBe!("OFF");

#[macro_export]
macro_rules! impl_monostate_member {
    ($strct:ident.$name:ident = $value:expr) => {
        ::paste::paste! {
            #[cfg_mixin(feature = "py")]
            #[cfg_attr(feature = "py", pyo3::pymethods)]
            impl $strct {
                #[on(setter)]
                pub fn [<set_ $name>](&mut self, on: bool) {
                    self.$name = if on { Some(::monostate::MustBe!($value)) } else { None };
                }

                #[on(getter)]
                pub fn [<get_ $name>](&self) -> bool {
                    self.$name.is_some()
                }
            }
        }
    };
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ApplicationType {
    Handling,
    Arc,
    Spot,
    Dispense
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum PalletizingMode {
    ZERODN,
    ZEROUP,
    PSPIDN,
    PSPIUP,
    MSPIDN,
    MSPIUP
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct FrameData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
    pub p: f32,
    pub r: f32,
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct Configuration {
    #[serde(rename = "UToolNumber")]
    pub u_tool_number: u8,
    #[serde(rename = "UFrameNumber")]
    pub u_frame_number: u8,
    #[serde(rename = "Front")]
    pub front: i8,
    #[serde(rename = "Up")]
    pub up: i8,
    #[serde(rename = "Left")]
    pub left: i8,
    #[serde(rename = "Flip")]
    pub flip: i8,
    #[serde(rename = "Turn4")]
    pub turn4: i8,
    #[serde(rename = "Turn5")]
    pub turn5: i8,
    #[serde(rename = "Turn6")]
    pub turn6: i8,
}

impl Default for Configuration {
    fn default() -> Self {
        Self {
            u_tool_number: 1,
            u_frame_number: 1,
            front: 1,
            up: 1,
            left: 1,
            flip: 0,
            turn4: 0,
            turn5: 0,
            turn6: 0,
        }
    }
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[serde(rename_all = "PascalCase")]
pub struct Position {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
    pub p: f32,
    pub r: f32,
    pub ext1: f32,
    pub ext2: f32,
    pub ext3: f32,
}

impl Default for Position {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
            p: 0.0,
            r: 0.0,
            ext1: 0.0,
            ext2: 0.0,
            ext3: 0.0,
        }
    }
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(str, from_py_object, get_all, set_all))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
#[serde(rename_all = "PascalCase")]
pub struct JointAngles {
    pub j1: f32,
    pub j2: f32,
    pub j3: f32,
    pub j4: f32,
    pub j5: f32,
    pub j6: f32,
    pub j7: f32,
    pub j8: f32,
    pub j9: f32,
}

// #[cfg(feature = "py")]
// #[pyo3::pymethods]
#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pymethods)]
impl JointAngles {
    #[on(new)]
    #[on(pyo3(signature = (format, template, j1, j2, j3, j4, j5, j6, j7=0.0, j8=0.0, j9=0.0)))]
    pub fn new9(
        format: JointFormat,
        template: JointTemplate,
        j1: f32,
        j2: f32,
        j3: f32,
        j4: f32,
        j5: f32,
        j6: f32,
        j7: f32,
        j8: f32,
        j9: f32,
    ) -> Self {
        let slf = Self {
            j1,
            j2,
            j3,
            j4,
            j5,
            j6,
            j7,
            j8,
            j9,
        };
        JointFormat::FanucDeg.convert_from(format, template, slf)
    }

    #[cfg(off)]
    pub fn new6(
        format: JointFormat,
        template: JointTemplate,
        j1: f32,
        j2: f32,
        j3: f32,
        j4: f32,
        j5: f32,
        j6: f32
    ) -> Self {
        let slf = Self {
            j1,
            j2,
            j3,
            j4,
            j5,
            j6,
            j7: 0.0,
            j8: 0.0,
            j9: 0.0
        };
        JointFormat::FanucDeg.convert_from(format, template, slf)
    }

    pub fn as_array(&self) -> [f32; 9] {
        [
            self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7, self.j8, self.j9,
        ]
    }
}

impl std::fmt::Display for JointAngles {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let json_string = serde_json::to_string_pretty(self).map_err(|_| std::fmt::Error)?;
        write!(f, "{}", json_string)
    }
}

impl JointRepr for JointAngles {
    fn to_deg(mut self, mask: &[JointType]) -> Self {
        if mask.len() > 0 && mask[0] == JointType::Rotary {
            self.j1 = self.j1.to_degrees();
        }
        if mask.len() > 1 && mask[1] == JointType::Rotary {
            self.j2 = self.j2.to_degrees();
        }
        if mask.len() > 2 && mask[2] == JointType::Rotary {
            self.j3 = self.j3.to_degrees();
        }
        if mask.len() > 3 && mask[3] == JointType::Rotary {
            self.j4 = self.j4.to_degrees();
        }
        if mask.len() > 4 && mask[4] == JointType::Rotary {
            self.j5 = self.j5.to_degrees();
        }
        if mask.len() > 5 && mask[5] == JointType::Rotary {
            self.j6 = self.j6.to_degrees();
        }
        if mask.len() > 6 && mask[6] == JointType::Rotary {
            self.j7 = self.j7.to_degrees();
        }
        if mask.len() > 7 && mask[7] == JointType::Rotary {
            self.j8 = self.j8.to_degrees();
        }
        if mask.len() > 8 && mask[8] == JointType::Rotary {
            self.j9 = self.j9.to_degrees();
        }
        self
    }

    fn to_rad(mut self, mask: &[JointType]) -> Self {
        if mask.len() > 0 && mask[0] == JointType::Rotary {
            self.j1 = self.j1.to_radians();
        }
        if mask.len() > 1 && mask[1] == JointType::Rotary {
            self.j2 = self.j2.to_radians();
        }
        if mask.len() > 2 && mask[2] == JointType::Rotary {
            self.j3 = self.j3.to_radians();
        }
        if mask.len() > 3 && mask[3] == JointType::Rotary {
            self.j4 = self.j4.to_radians();
        }
        if mask.len() > 4 && mask[4] == JointType::Rotary {
            self.j5 = self.j5.to_radians();
        }
        if mask.len() > 5 && mask[5] == JointType::Rotary {
            self.j6 = self.j6.to_radians();
        }
        if mask.len() > 6 && mask[6] == JointType::Rotary {
            self.j7 = self.j7.to_radians();
        }
        if mask.len() > 7 && mask[7] == JointType::Rotary {
            self.j8 = self.j8.to_radians();
        }
        if mask.len() > 8 && mask[8] == JointType::Rotary {
            self.j9 = self.j9.to_radians();
        }
        self
    }

    fn to_abs(mut self) -> Self {
        self.j3 += self.j2;
        self
    }

    fn to_fanuc(mut self) -> Self {
        self.j3 -= self.j2;
        self
    }

    fn len(&self) -> usize {
        9
    }

    fn to_array<const N: usize, D: JointValue>(
        self,
        _fill_missing_nan: bool,
    ) -> Result<[D; N], crate::joints::JointDataSizeError> {
        if N > 9 {
            return Err(crate::joints::JointDataSizeError(9));
        }
        let mut arr: [D; N] = [D::nan(); N];
        let joints = [
            self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7, self.j8, self.j9,
        ];
        for i in 0..N {
            arr[i] = D::from_f32(joints[i]);
        }
        Ok(arr)
    }
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum LcbType {
    TB,
    DB,
    TA,
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize_repr, Deserialize_repr, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PortType {
    DOUT = 1,
    ROUT = 2,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct LocalConditionBlock {
    #[on(pyo3(get, set))]
    #[serde(rename = "LCBType")]
    pub lcb_type: LcbType,
    #[on(pyo3(get, set))]
    #[serde(rename = "LCBValue")]
    pub lcb_value: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "PortType")]
    pub port_type: PortType,
    #[on(pyo3(get, set))]
    #[serde(rename = "portNumber")]
    pub port_number: u16,
    #[on(pyo3(get, set))]
    #[serde(rename = "portValue")]
    pub port_value: OnOff,
}

#[cfg_mixin(feature = "py")]
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq)]
pub struct OffsetRegisterNumbers {
    #[on(pyo3(get, set))]
    #[serde(
        rename = "OffsetPRNumber",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub offset: Option<u16>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "VisionPRNumber",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub vision_offset: Option<u16>,
    #[on(pyo3(get, set))]
    #[serde(
        rename = "ToolOffsetPRNumber",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub tool_offset: Option<u16>,
}

impl Default for OffsetRegisterNumbers {
    fn default() -> Self {
        Self {
            offset: None,
            vision_offset: None,
            tool_offset: None,
        }
    }
}

#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum TermType {
    FINE,
    /// Continuous motion
    CNT,
    /// CR with a value from 1 to 100
    CR,
}

/// Represents different types of speed measurements.
///
/// This enum provides various units of speed that can be used
/// to specify movement or duration in different contexts.
///
/// # Variants
///
/// * `MMSec` - Represents speed in millimeters per second (mm/sec).
/// * `InchMin` - Represents speed in inches per second.
/// * `Time` - Represents time in 0.1 second increments.
/// * `MilliSeconds` - Represents time in milliseconds (0.001 seconds).
#[cfg_attr(feature = "py", pyo3::pyclass(from_py_object))]
#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum SpeedType {
    #[serde(rename = "mmSec")]
    MMSec, // Speed in millimeters per second (mm/sec).
    #[serde(rename = "InchMin")]
    InchMin, // Speed in inches per second.
    #[serde(rename = "Time")]
    TimeSec, // Time in 0.1 second increments.
    #[serde(rename = "mSec")]
    MilliSeconds, // Time in milliseconds (0.001 seconds).
}

#[cfg(feature = "py")]
pub mod py {
    use super::*;
    use pyo3::prelude::*;

    pub fn register(parent_module: &Bound<'_, PyModule>) -> PyResult<()> {
        parent_module.add_class::<OnOff>()?;
        parent_module.add_class::<ApplicationType>()?;
        parent_module.add_class::<FrameData>()?;
        parent_module.add_class::<Configuration>()?;
        parent_module.add_class::<Position>()?;
        parent_module.add_class::<JointAngles>()?;
        parent_module.add_class::<LcbType>()?;
        parent_module.add_class::<PortType>()?;
        parent_module.add_class::<LocalConditionBlock>()?;
        parent_module.add_class::<OffsetRegisterNumbers>()?;
        parent_module.add_class::<TermType>()?;
        parent_module.add_class::<SpeedType>()?;

        Ok(())
    }
}
