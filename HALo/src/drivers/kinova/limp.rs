//! Explicit zero commanded torque, with the SDK gravity vector suppressed.
//! Vendor protections remain enabled. Physical backdrivability needs robot validation.
use super::sdk::KinovaSdk;

pub trait TorqueApi {
    fn supported(&self) -> bool;
    fn erase(&self) -> Result<(), String>;
    fn zero_velocity(&self) -> Result<(), String>;
    fn direct_torque(&self) -> Result<(), String>;
    fn gravity(&self, vector: [f32; 3]) -> Result<(), String>;
    fn switch(&self, torque: bool) -> Result<(), String>;
    fn mode(&self) -> Result<bool, String>;
    fn zero_torque(&self) -> Result<(), String>;
    fn angular(&self) -> Result<(), String>;
}

#[derive(Default)]
pub struct LimpControl {
    pub active: bool,
    pub inhibited: bool,
    pub error: Option<String>,
}
impl LimpControl {
    pub fn estop(&mut self, api: &impl TorqueApi, gravity: [f32; 3]) -> Result<(), String> {
        if !api.supported() {
            let mut errors = vec![
                "SDK torque functions unavailable; limp stop could not be confirmed".to_owned(),
            ];
            if let Err(error) = api.erase() {
                errors.push(error);
            }
            if let Err(error) = api.zero_velocity() {
                errors.push(error);
            }
            self.inhibited = true;
            self.error = Some(errors.join("; "));
            return Err(self.error.clone().unwrap());
        }
        let result = self.set(api, true, gravity);
        // An unsuccessful stop may restore the SDK controller internally, but
        // must never authorize queued motion. Only explicit restore releases it.
        self.inhibited = true;
        if let Err(error) = &result {
            self.error = Some(error.clone());
        }
        result
    }
    pub fn set(
        &mut self,
        api: &impl TorqueApi,
        enabled: bool,
        gravity: [f32; 3],
    ) -> Result<(), String> {
        if !api.supported() {
            return Err("SDK does not support limp mode".into());
        }
        self.inhibited = true;
        let result = if enabled {
            (|| {
                api.erase()?;
                api.zero_velocity()?;
                api.direct_torque()?;
                api.zero_torque()?;
                api.gravity([0.0; 3])?;
                api.switch(true)?;
                if !api.mode()? {
                    return Err("firmware rejected torque mode".into());
                }
                api.zero_torque()
            })()
        } else {
            self.restore(api, gravity)
        };
        match result {
            Ok(()) => {
                self.active = enabled;
                self.inhibited = enabled;
                self.error = None;
                Ok(())
            }
            Err(error) => {
                // Do not release the motion interlock until every restore call succeeds.
                let restore = self.restore(api, gravity);
                self.active = false;
                self.inhibited = restore.is_err();
                let error = match restore {
                    Ok(()) => error,
                    Err(e) => format!("{error}; restore failed: {e}"),
                };
                self.error = Some(error.clone());
                Err(error)
            }
        }
    }
    fn restore(&self, api: &impl TorqueApi, gravity: [f32; 3]) -> Result<(), String> {
        // Clear old motion before handing control back to the position controller.
        api.erase()?;
        api.switch(false)?;
        if api.mode()? {
            return Err("firmware did not leave torque mode".into());
        }
        api.gravity(gravity)?;
        api.angular()?;
        api.zero_velocity()
    }
    pub fn pulse(&mut self, api: &impl TorqueApi) {
        if self.active {
            if let Err(e) = api.zero_torque() {
                self.error = Some(e);
            }
        }
    }
    pub fn verify(&mut self, api: &impl TorqueApi) {
        if self.active {
            match api.mode() {
                Ok(true) => {}
                Ok(false) => {
                    self.active = false;
                    self.error = Some("firmware left torque mode; motion remains blocked until limp mode is disabled".into());
                }
                Err(e) => self.error = Some(e),
            }
        }
    }
}
impl TorqueApi for KinovaSdk {
    fn supported(&self) -> bool {
        self.limp_supported()
    }
    fn erase(&self) -> Result<(), String> {
        self.erase_all_trajectories().map_err(|e| e.to_string())
    }
    fn zero_velocity(&self) -> Result<(), String> {
        self.send_basic_trajectory(super::ffi::angular_velocity_point([0.0; 6]))
            .map_err(|e| e.to_string())
    }
    fn direct_torque(&self) -> Result<(), String> {
        self.set_direct_torque().map_err(|e| e.to_string())
    }
    fn gravity(&self, v: [f32; 3]) -> Result<(), String> {
        self.set_gravity_vector(v).map_err(|e| e.to_string())
    }
    fn switch(&self, v: bool) -> Result<(), String> {
        self.switch_torque(v).map_err(|e| e.to_string())
    }
    fn mode(&self) -> Result<bool, String> {
        self.torque_mode().map_err(|e| e.to_string())
    }
    fn zero_torque(&self) -> Result<(), String> {
        self.send_zero_torque().map_err(|e| e.to_string())
    }
    fn angular(&self) -> Result<(), String> {
        self.set_angular_control().map_err(|e| e.to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::{Cell, RefCell};
    #[derive(Default)]
    struct Api {
        calls: RefCell<Vec<String>>,
        mode: Cell<bool>,
        reject: Cell<bool>,
        restore_fail: Cell<bool>,
        unsupported: Cell<bool>,
    }
    impl Api {
        fn call(&self, s: &str) -> Result<(), String> {
            self.calls.borrow_mut().push(s.into());
            Ok(())
        }
    }
    impl TorqueApi for Api {
        fn supported(&self) -> bool {
            !self.unsupported.get()
        }
        fn erase(&self) -> Result<(), String> {
            self.call("erase")
        }
        fn zero_velocity(&self) -> Result<(), String> {
            self.call("zero_velocity")
        }
        fn direct_torque(&self) -> Result<(), String> {
            self.call("direct")
        }
        fn gravity(&self, v: [f32; 3]) -> Result<(), String> {
            self.call(if v == [0.0; 3] {
                "gravity_off"
            } else {
                "gravity_restore"
            })
        }
        fn switch(&self, v: bool) -> Result<(), String> {
            self.call(if v { "torque" } else { "position" })?;
            if !v && self.restore_fail.get() {
                return Err("restore".into());
            }
            self.mode.set(v && !self.reject.get());
            Ok(())
        }
        fn mode(&self) -> Result<bool, String> {
            Ok(self.mode.get())
        }
        fn zero_torque(&self) -> Result<(), String> {
            self.call("zero_torque")
        }
        fn angular(&self) -> Result<(), String> {
            self.call("angular")
        }
    }
    #[test]
    fn explicit_limp_cancels_motion_verifies_mode_and_restores_gravity() {
        let api = Api::default();
        let mut c = LimpControl::default();
        c.set(&api, true, [0., 0., -9.81]).unwrap();
        assert!(c.active && c.inhibited);
        assert_eq!(
            *api.calls.borrow(),
            [
                "erase",
                "zero_velocity",
                "direct",
                "zero_torque",
                "gravity_off",
                "torque",
                "zero_torque"
            ]
        );
        c.pulse(&api);
        assert_eq!(api.calls.borrow().last().unwrap(), "zero_torque");
        c.set(&api, false, [0., 0., -9.81]).unwrap();
        assert!(!c.active && !c.inhibited);
        assert_eq!(
            &api.calls.borrow()[8..],
            [
                "erase",
                "position",
                "gravity_restore",
                "angular",
                "zero_velocity"
            ]
        );
    }
    #[test]
    fn firmware_rejection_and_uncertain_restore_never_claim_limp_or_allow_motion() {
        let api = Api::default();
        api.reject.set(true);
        api.restore_fail.set(true);
        let mut c = LimpControl::default();
        assert!(c.set(&api, true, [0., 0., -9.81]).is_err());
        assert!(!c.active && c.inhibited && c.error.is_some());
        api.restore_fail.set(false);
        c.set(&api, false, [0., 0., -9.81]).unwrap();
        assert!(!c.inhibited);
    }
    #[test]
    fn firmware_fallback_keeps_motion_interlock_latched() {
        let api = Api::default();
        let mut c = LimpControl::default();
        c.set(&api, true, [0., 0., -9.81]).unwrap();
        api.mode.set(false);
        c.verify(&api);
        assert!(!c.active && c.inhibited && c.error.is_some());
    }
    #[test]
    fn estop_clears_motion_and_latches_even_when_firmware_rejects_torque() {
        let api = Api::default();
        let mut control = LimpControl::default();
        control.estop(&api, [0., 0., -9.81]).unwrap();
        assert!(control.active && control.inhibited);
        assert_eq!(api.calls.borrow()[0], "erase");
        control.set(&api, false, [0., 0., -9.81]).unwrap();
        api.reject.set(true);
        assert!(control.estop(&api, [0., 0., -9.81]).is_err());
        assert!(!control.active && control.inhibited && control.error.is_some());
        control.set(&api, false, [0., 0., -9.81]).unwrap();
        assert!(!control.inhibited);
    }
    #[test]
    fn estop_still_cancels_motion_without_optional_torque_symbols() {
        let api = Api::default();
        api.unsupported.set(true);
        let mut control = LimpControl::default();
        let error = control.estop(&api, [0., 0., -9.81]).unwrap_err();
        assert!(error.contains("could not be confirmed"));
        assert_eq!(*api.calls.borrow(), ["erase", "zero_velocity"]);
        assert!(control.inhibited && !control.active);
    }
}
