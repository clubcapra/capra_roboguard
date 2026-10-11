use super::{driver::FieldDescriptor, error::DriverError};
use serde_json::Value;

pub fn command(payload: &Value, fields: &[FieldDescriptor]) -> Result<(), DriverError> {
    let invalid = |msg: String| DriverError::InvalidCommand(msg);
    let object = payload
        .as_object()
        .ok_or_else(|| invalid("expected an object".into()))?;
    if object.is_empty() {
        return Err(invalid("empty command".into()));
    }
    for (key, value) in object {
        let field = fields
            .iter()
            .find(|f| &f.name == key)
            .ok_or_else(|| invalid(format!("unknown field '{key}'")))?;
        if field.standalone && object.len() != 1 {
            return Err(invalid(format!("'{key}' must be sent alone")));
        }
        let valid = match field.type_name.as_str() {
            "bool" => value.is_boolean(),
            "String" | "str" => value.is_string(),
            "f32" => value
                .as_f64()
                .is_some_and(|v| v.is_finite() && v.abs() <= f32::MAX as f64),
            "f64" => value.as_f64().is_some_and(f64::is_finite),
            "u8" => value.as_u64().is_some_and(|v| v <= u8::MAX as u64),
            "u16" => value.as_u64().is_some_and(|v| v <= u16::MAX as u64),
            "u32" => value.as_u64().is_some_and(|v| v <= u32::MAX as u64),
            "u64" => value.as_u64().is_some(),
            "i8" => value.as_i64().is_some_and(|v| i8::try_from(v).is_ok()),
            "i16" => value.as_i64().is_some_and(|v| i16::try_from(v).is_ok()),
            "i32" => value.as_i64().is_some_and(|v| i32::try_from(v).is_ok()),
            "i64" => value.as_i64().is_some(),
            _ => true, // Driver-specific structured types are validated by the driver.
        };
        if !valid {
            return Err(invalid(format!("'{key}' must be {}", field.type_name)));
        }
    }
    Ok(())
}
