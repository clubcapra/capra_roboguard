use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet},
    io,
    path::PathBuf,
};

#[derive(Clone, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Assignments {
    pub sensors: BTreeMap<String, [u16; 2]>,
}

pub struct PortStore {
    pub assignments: Assignments,
    path: Option<PathBuf>,
    base: u16,
    reserved: BTreeSet<u16>,
}

impl PortStore {
    pub fn new(base: u16, path: Option<PathBuf>, reserved: BTreeSet<u16>) -> io::Result<Self> {
        let assignments = match path.as_ref() {
            Some(p) if p.exists() => serde_json::from_slice(&std::fs::read(p)?)?,
            _ => Assignments::default(),
        };
        let store = Self {
            assignments,
            path,
            base,
            reserved,
        };
        store.validate(&store.assignments)?;
        Ok(store)
    }

    fn validate(&self, assignments: &Assignments) -> io::Result<()> {
        let mut used = self.reserved.clone();
        for (id, ports) in &assignments.sensors {
            if id.is_empty()
                || !id
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b == b'_' || b == b'-')
            {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "invalid sensor ID",
                ));
            }
            for port in ports {
                if *port == 0 || !used.insert(*port) {
                    return Err(io::Error::new(
                        io::ErrorKind::AddrInUse,
                        format!("port {port} conflicts ({id})"),
                    ));
                }
            }
        }
        Ok(())
    }

    pub fn set(&mut self, id: &str, ports: [u16; 2]) -> io::Result<()> {
        let mut next = self.assignments.clone();
        next.sensors.insert(id.into(), ports);
        self.validate(&next)?;
        if let Some(path) = &self.path {
            super::config::atomic_write(path, &serde_json::to_vec_pretty(&next)?)?;
        }
        self.assignments = next;
        Ok(())
    }

    pub fn reserve_external(&mut self, ports: &[u16]) -> io::Result<()> {
        if ports.iter().any(|p| {
            *p == 0
                || self
                    .assignments
                    .sensors
                    .values()
                    .flatten()
                    .any(|used| used == p)
        }) {
            return Err(io::Error::new(
                io::ErrorKind::AddrInUse,
                "controller port conflicts with a reserved sensor port",
            ));
        }
        self.reserved.extend(ports.iter().copied());
        Ok(())
    }

    pub fn ensure(&mut self, id: &str) -> io::Result<[u16; 2]> {
        // Reservations belong to identities, including hardware currently absent.
        // Never renumber an existing device to accommodate a topology change or
        // an occupied socket; only an explicit set() can change its assignment.
        if let Some(ports) = self.assignments.sensors.get(id) {
            return Ok(*ports);
        }
        let used: BTreeSet<u16> = self
            .assignments
            .sensors
            .values()
            .flatten()
            .copied()
            .chain(self.reserved.iter().copied())
            .collect();
        let mut data = self.base;
        loop {
            let command = data
                .checked_add(1)
                .ok_or_else(|| io::Error::other("UDP port range exhausted"))?;
            if data > 0 && !used.contains(&data) && !used.contains(&command) {
                self.set(id, [data, command])?;
                return Ok([data, command]);
            }
            data = data
                .checked_add(2)
                .ok_or_else(|| io::Error::other("UDP port range exhausted"))?;
        }
    }
}
