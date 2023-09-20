// SPDX-FileCopyrightText: Copyright (c) 2023 by Rivos Inc.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#![cfg_attr(not(test), no_std)]

use core::sync::atomic::{AtomicUsize, Ordering};

// BAR Attributes
const BAR_ATTRIBUTES_MASK: u64 = 0xf;
const BASE_ADDRESS_MASK: u64 = 0x1;
const BASE_ADDRESS_MEM: u64 = 0x0;
const MEM_LIMIT_MASK: u64 = 6;
const MEM_LIMIT_32: u64 = 0;
const MEM_LIMIT_1M: u64 = 2;
const MEM_LIMIT_64: u64 = 4;
const BAR_PREFETCHABLE: u64 = 8;

// Command bits
pub const PCI_COMMAND_IO: u16 = 1;
pub const PCI_COMMAND_MEMORY: u16 = 2;
pub const PCI_COMMAND_BUS_MASTER: u16 = 4;

// Bit set in HeaderType register to indicate a multi-function device
const MULTI_FUNCTION: u8 = 0x80;

const MAX_BUS: Bus = Bus(255);
const MAX_DEV: Dev = Dev(31);
const MAX_FUNC: Func = Func(7);

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Description {
    Unknown,
    HostBridge,
    PciPciBridge,
    NvmeController,
    Ethernet,
    Serial,
}

#[derive(Copy, Clone, Debug)]
pub struct DeviceInfo {
    pub desc: Description,
    pub class: ClassCode,
    pub subclass: u8,
    pub prog_if: u8,
}

impl DeviceInfo {
    fn desc_of(class: ClassCode, subclass: u8, prog_if: u8) -> Description {
        match (class, subclass, prog_if) {
            (ClassCode::MassStorage, 8, 2) => Description::NvmeController,
            (ClassCode::Bridge, 4, 0) => Description::PciPciBridge,
            (ClassCode::Bridge, 0, 0) => Description::HostBridge,
            (ClassCode::Network, 0, 0) => Description::Ethernet,
            (ClassCode::SimpleCommunications, 0, _) => Description::Serial,
            // TODO: Add more
            _ => Description::Unknown,
        }
    }

    pub fn new(class: u8, subclass: u8, prog_if: u8) -> Self {
        Self {
            class: class.into(),
            subclass,
            prog_if,
            desc: Self::desc_of(class.into(), subclass, prog_if),
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum ClassCode {
    Unknown,
    MassStorage,
    Network,
    Display,
    Multimedia,
    Memory,
    Bridge,
    SimpleCommunications,
    BaseSystemPeripheral,
    Input,
    DockingStation,
    Processors,
    SerialBus,
    Wireless,
    IntelligentIO,
    SatelliteCommunications,
    EncryptionDecryption,
    DaqAndSignalProcessing,
    Accelerators,
    NonEssential,
    Reserved,
}

impl From<u8> for ClassCode {
    fn from(value: u8) -> Self {
        match value {
            0 => ClassCode::Unknown,
            1 => ClassCode::MassStorage,
            2 => ClassCode::Network,
            3 => ClassCode::Display,
            4 => ClassCode::Multimedia,
            5 => ClassCode::Memory,
            6 => ClassCode::Bridge,
            7 => ClassCode::SimpleCommunications,
            8 => ClassCode::BaseSystemPeripheral,
            9 => ClassCode::Input,
            10 => ClassCode::DockingStation,
            11 => ClassCode::Processors,
            12 => ClassCode::SerialBus,
            13 => ClassCode::Wireless,
            14 => ClassCode::IntelligentIO,
            15 => ClassCode::SatelliteCommunications,
            16 => ClassCode::EncryptionDecryption,
            17 => ClassCode::DaqAndSignalProcessing,
            18 => ClassCode::Accelerators,
            19 => ClassCode::NonEssential,
            _ => ClassCode::Reserved,
        }
    }
}

#[derive(Debug, Copy, Clone)]
pub enum ResourceType {
    Io,
    Memory32,
    Memory64,
}

#[derive(Debug, Copy, Clone)]
pub struct Resource {
    pub bdf: Bdf,
    idx: Register,
    pub ty: ResourceType,
    pub base: Option<u64>,
    pub size: u64,
    pub alignment: u64,
    pub limit: u64,
    pub prefetchable: bool,
    pub bridge: bool,
}

pub struct PciEcamCfgOps {
    pub segment_id: usize,
    base: AtomicUsize,
}

impl PciEcamCfgOps {
    pub const fn new(segment_id: usize, base: usize) -> Self {
        Self {
            segment_id,
            base: AtomicUsize::new(base as usize),
        }
    }

    /// # Safety
    ///
    /// It is up to the user to ensure that the same PCIe ECAM region
    /// was actually moved in some platform-specific manner.
    pub unsafe fn rebase(&self, new_base: *mut u8) {
        self.base.store(new_base as usize, Ordering::Release);
    }

    fn addr(&self, bdf: &Bdf, register: u16) -> usize {
        self.base
            .load(Ordering::Acquire)
            .wrapping_add((bdf.bus.val() as usize) << 20)
            .wrapping_add((bdf.dev.val() as usize) << 15)
            .wrapping_add((bdf.func.val() as usize) << 12)
            .wrapping_add(register as usize)
    }

    fn read8(&self, bdf: &Bdf, register: u16) -> u8 {
        unsafe { core::ptr::read_volatile(self.addr(bdf, register) as *const u8) }
    }

    fn read16(&self, bdf: &Bdf, register: u16) -> u16 {
        unsafe { core::ptr::read_volatile(self.addr(bdf, register) as *const u16) }
    }

    fn read32(&self, bdf: &Bdf, register: u16) -> u32 {
        unsafe { core::ptr::read_volatile(self.addr(bdf, register) as *const u32) }
    }

    fn write8(&self, bdf: &Bdf, register: u16, value: u8) {
        unsafe { core::ptr::write_volatile(self.addr(bdf, register) as *mut u8, value) }
    }

    fn write16(&self, bdf: &Bdf, register: u16, value: u16) {
        unsafe { core::ptr::write_volatile(self.addr(bdf, register) as *mut u16, value) }
    }

    fn write32(&self, bdf: &Bdf, register: u16, value: u32) {
        unsafe { core::ptr::write_volatile(self.addr(bdf, register) as *mut u32, value) }
    }
}

// Add some newtypes to these raw numbers so we can't mix these up
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct Bus(u8);

impl Bus {
    pub fn val(&self) -> u8 {
        self.0
    }
}

impl From<u8> for Bus {
    fn from(val: u8) -> Self {
        Self(val)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct Func(u8);

impl Func {
    pub fn val(&self) -> u8 {
        self.0
    }
}

impl From<u8> for Func {
    fn from(val: u8) -> Self {
        Self(val)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub struct Dev(u8);

impl Dev {
    pub fn val(&self) -> u8 {
        self.0
    }
}

impl From<u8> for Dev {
    fn from(val: u8) -> Self {
        Self(val)
    }
}

// PCI header types
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum HeaderType {
    Normal = 0,
    Bridge = 1,
    Cardbus = 2,
}

impl TryFrom<u8> for HeaderType {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(HeaderType::Normal),
            1 => Ok(HeaderType::Bridge),
            2 => Ok(HeaderType::Cardbus),
            _ => Err("Invalid header type value"),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Register {
    VendorId = 0,
    DeviceId = 2,
    Command = 4,
    Status = 6,
    RevisionId = 8,
    ClassCode = 9,
    CacheLineSize = 0xc,
    LatencyTimer = 0xd,
    HeaderType = 0xe,
    Bist = 0xf,
    Bar0 = 0x10,
    Bar1 = 0x14,

    // Endpoint-specific (Header type 0)
    Bar2 = 0x18,
    Bar3 = 0x1c,
    Bar4 = 0x20,
    Bar5 = 0x24,
    CardbusPointer = 0x28,
    SubsystemVendorId = 0x2c,
    SubsystemId = 0x2e,
    ExpansionRomBar = 0x30,
    CapabilityPointer = 0x34,
    InterruptLine = 0x3c,
    InterruptPin = 0x3d,
    MinGrant = 0x3e,
    MaxLatency = 0x3f,

    // Bridge-specific (Header type 1)
    SecondaryBus = 0x19,
    SubordinateBus = 0x1a,
    SecondaryLatency = 0x1b,
    IoLimit = 0x1d,
    SecondaryStatus = 0x1e,
    MemoryLimit = 0x22,
    PrefMemLimit = 0x26,

    ExtendedCapHeader = 0x100,
}

// Cheap hack to have values that appear to alias each other
#[allow(non_upper_case_globals)]
impl Register {
    pub const PrimaryBus: Register = Register::Bar2;
    pub const IoBase: Register = Register::Bar3;
    pub const MemoryBase: Register = Register::Bar4;
    pub const PrefMemBase: Register = Register::Bar5;
    pub const PrefBaseUpper: Register = Register::CardbusPointer;
    pub const PrefLimitUpper: Register = Register::SubsystemVendorId;
    pub const BridgeControl: Register = Register::MinGrant;
}

const HEADER_TYPE_OFFSET: u16 = Register::HeaderType as u16;
const CLASS_CODE_OFFSET: u16 = Register::ClassCode as u16 + 2;
const SUBCLASS_OFFSET: u16 = Register::ClassCode as u16 + 1;
const PROG_IF_OFFSET: u16 = Register::ClassCode as u16;

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Bdf {
    pub bus: Bus,
    pub dev: Dev,
    pub func: Func,
}

impl Bdf {
    pub fn to_usize(bdf: &Bdf) -> usize {
        (bdf.bus.val() as usize) << 8 | (bdf.dev.val() as usize) << 3 | (bdf.func.val() as usize)
    }

    pub fn from_usize(bdf: usize) -> Self {
        Bdf {
            bus: (((bdf >> 8) & 0xff) as u8).into(),
            dev: (((bdf >> 3) & 0x1f) as u8).into(),
            func: ((bdf & 0x7) as u8).into(),
        }
    }

    fn new(bus: Bus, dev: Dev, func: Func) -> Self {
        assert!(func <= MAX_FUNC);
        assert!(dev <= MAX_DEV);
        Self { bus, dev, func }
    }

    // Construct by bus number
    pub fn from_bus(bus: Bus) -> Self {
        Self::new(bus, Dev(0), Func(0))
    }

    // Return the next device (skip the rest of the functions in this device)
    fn next_dev(&self) -> Option<Bdf> {
        if self.dev < MAX_DEV {
            Some(Bdf::new(self.bus, Dev(self.dev.0 + 1), Func(0)))
        } else {
            None
        }
    }

    // Return the next bdf
    fn next(&self) -> Option<Bdf> {
        if self.func < MAX_FUNC {
            Some(Bdf::new(self.bus, self.dev, Func(self.func.0 + 1)))
        } else if self.dev < MAX_DEV {
            Some(Bdf::new(self.bus, Dev(self.dev.0 + 1), Func(0)))
        } else if self.bus < MAX_BUS {
            Some(Bdf::new(Bus(self.bus.0 + 1), Dev(0), Func(0)))
        } else {
            None
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct PciCap {
    pub capability_id: u8,
    pub base_register: u16,
}

#[derive(Copy, Clone, Debug)]
pub struct PcieExtCap {
    pub capability_id: u16,
    pub version: u8,
    pub base_register: u16,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum CapabilityId {
    PciPm = 0x1,
    Agp = 0x2,
    Vpd = 0x3,
    SlotId = 0x4,
    Msi = 0x5,
    PciExpress = 0x10,
    ExtendedAllocation = 0x14,
}

#[repr(u16)]
#[derive(Copy, Clone)]
pub enum ExtendedCapabilityId {
    Aer = 0x1,
    VendorSpecific = 0x9,
    Acs = 0xd,
    ResizableBar = 0x15,
    Ltr = 0x18,
    Secondary = 0x19,
    L1PmSubstates = 0x1e,
    Doe = 0x2e,
}

const BARS: [Register; 6] = [
    Register::Bar0,
    Register::Bar1,
    Register::Bar2,
    Register::Bar3,
    Register::Bar4,
    Register::Bar5,
];

// Public fns
impl<'a> Device<'a> {
    pub fn cfg_read8(&self, register: u16) -> u8 {
        self.ops.read8(&self.bdf, register)
    }
    pub fn cfg_read16(&self, register: u16) -> u16 {
        self.ops.read16(&self.bdf, register)
    }
    pub fn cfg_read32(&self, register: u16) -> u32 {
        self.ops.read32(&self.bdf, register)
    }
    pub fn cfg_write8(&self, register: u16, value: u8) {
        self.ops.write8(&self.bdf, register, value);
    }
    pub fn cfg_write16(&self, register: u16, value: u16) {
        self.ops.write16(&self.bdf, register, value);
    }
    pub fn cfg_write32(&self, register: u16, value: u32) {
        self.ops.write32(&self.bdf, register, value);
    }
    pub fn resources(&self) -> &[Option<Resource>; 6] {
        &self.resources
    }

    // flatten() doesn't return mutable data
    #[allow(clippy::manual_flatten)]
    // Set the base of a given resource (in memory only)
    pub fn assign_resource(&mut self, res: &Resource) {
        let mut assigned = false;

        for resource in &mut self.resources {
            if resource.is_some() {
                if res.idx == resource.unwrap().idx {
                    *resource = Some(*res);
                    assigned = true;
                    break;
                }
            }
        }

        if !assigned {
            panic!("Invalid idx in assign_resource!");
        }
    }

    // Write all of the resources to config space
    pub fn store_resources(&self) {
        // Device BARs are simpler, handle them early.
        if !self.is_bridge() {
            for resource in self.resources.iter().flatten() {
                if let Some(base) = resource.base {
                    let base_lo = (base & 0xffff_ffff) as u32;
                    self.cfg_write32(resource.idx as u16, base_lo);

                    if matches!(resource.ty, ResourceType::Memory64) {
                        let base_hi = ((base >> 32) & 0xffff_ffff) as u32;
                        self.cfg_write32(resource.idx as u16 + 4, base_hi);
                    }
                }
            }

            return;
        }

        for resource in self.resources[0..2].iter().flatten() {
            if let Some(base) = resource.base {
                match resource.idx {
                    Register::Bar0 => {
                        let base_lo = (base & 0xffff_ffff) as u32;
                        self.cfg_write32(resource.idx as u16, base_lo);

                        if matches!(resource.ty, ResourceType::Memory64) {
                            let base_hi = ((base >> 32) & 0xffff_ffff) as u32;
                            self.cfg_write32(resource.idx as u16 + 4, base_hi);
                        }
                    }
                    Register::Bar1 => {
                        let base_lo = (base & 0xffff_ffff) as u32;
                        self.cfg_write32(resource.idx as u16, base_lo);
                    }
                    _ => panic!("Bug in pcie library!"),
                }
            }
        }

        // NOTE: PCI bridge resources have no enable bit,
        // instead the disable logic is active if the base is
        // greater than the end. A safe strategy is then to
        // set the base to the limit, and the end to the limit
        // minus 1 alignment.
        for resource in self.resources[2..4].iter().flatten() {
            let (base, end) = if let Some(base) = resource.base {
                (base, resource.limit)
            } else {
                (resource.limit, resource.limit - resource.alignment)
            };

            match resource.idx {
                Register::MemoryBase => {
                    self.cfg_write16(Register::MemoryBase as u16, (base >> 16) as u16);
                    self.cfg_write16(Register::MemoryLimit as u16, (end >> 16) as u16);
                }
                Register::PrefMemBase => {
                    self.cfg_write16(Register::PrefMemBase as u16, (base >> 16) as u16);
                    self.cfg_write32(Register::PrefBaseUpper as u16, (base >> 32) as u32);
                    self.cfg_write16(Register::PrefMemLimit as u16, (end >> 16) as u16);
                    self.cfg_write32(Register::PrefLimitUpper as u16, (end >> 32) as u32);
                }
                _ => panic!("Bug in pcie library!"),
            }
        }
    }

    pub fn enable_resources(&self) {
        if self
            .probe_capability(CapabilityId::ExtendedAllocation)
            .is_some()
        {
            return;
        }

        let mut val: u16 = 0;
        for res in self.resources.iter().flatten() {
            val |= match res.ty {
                ResourceType::Io => PCI_COMMAND_IO,
                ResourceType::Memory32 | ResourceType::Memory64 => PCI_COMMAND_MEMORY,
            };
        }

        self.cfg_write16(Register::Command as u16, val);
    }

    // Read the resource requirements (or actual resources if EA) published by the device
    pub fn read_resources(&mut self) {
        // If the device supports extended allocation, then there are
        // already resources provided by the platform for the device,
        // they just have to be discovered here instead.
        if let Some(cap) = self.probe_capability(CapabilityId::ExtendedAllocation) {
            self.read_extended_allocation(cap);
            return;
        }

        // Clear the bus master, memory & io enable bits before reading BARs
        self.cfg_write16(
            Register::Command as u16,
            !(PCI_COMMAND_BUS_MASTER | PCI_COMMAND_MEMORY | PCI_COMMAND_IO),
        );

        if self.is_bridge() {
            self.read_bridge_resources();
        }

        let mut skip_next = false;
        for (idx, bar) in BARS.iter().take(self.max_bars()).enumerate() {
            if skip_next {
                skip_next = false;
                continue;
            }

            let resource = match self.size_bar(*bar) {
                None => continue,
                Some(r) => r,
            };

            self.resources[idx] = Some(resource);

            // a 64-bit memory resource consumes two physical BARs, so skip the next one
            if matches!(resource.ty, ResourceType::Memory64) {
                skip_next = true;
            }
        }
    }

    // Determine if the device has the given PCI capability
    pub fn probe_capability(&self, id: CapabilityId) -> Option<PciCap> {
        fn cap_id(cap: u16) -> u8 {
            (cap & 0xff) as u8
        }
        fn next_offset(cap: u16) -> u8 {
            (cap >> 8) as u8
        }

        let mut current_offset = self.cfg_read8(Register::CapabilityPointer as u16);
        while current_offset != 0 {
            let cap = self.cfg_read16(current_offset as u16);
            if cap == u16::MAX {
                return None;
            }

            if cap_id(cap) == id as u8 {
                return Some(PciCap {
                    capability_id: cap_id(cap),
                    base_register: current_offset as u16,
                });
            }

            current_offset = next_offset(cap);
        }

        None
    }

    // Determine if the device has the given PCIe extended capability
    pub fn probe_extended_capability(&self, id: ExtendedCapabilityId) -> Option<PcieExtCap> {
        fn ext_cap_id(cap: u32) -> u16 {
            (cap & 0xffff) as u16
        }
        fn next_offset(cap: u32) -> u16 {
            (cap >> 20 & 0xffc) as u16
        }
        fn version(cap: u32) -> u8 {
            (cap >> 16 & 0xf) as u8
        }

        let mut current_offset = Register::ExtendedCapHeader as u16;
        while current_offset != 0 {
            let cap = self.cfg_read32(current_offset);
            if cap == u32::MAX {
                return None;
            }

            if ext_cap_id(cap) == id as u16 {
                return Some(PcieExtCap {
                    capability_id: id as u16,
                    version: version(cap),
                    base_register: current_offset,
                });
            }

            current_offset = next_offset(cap);
        }

        None
    }
}

// Private fns
impl<'a> Device<'a> {
    fn max_bars(&self) -> usize {
        match self.is_bridge() {
            true => 2,
            false => 6,
        }
    }

    // Return the original value and the bits that "move" when the
    // register is written with all ones then all zeros.
    fn moving_bits32(&self, register: u16) -> (u32, u32) {
        let value = self.cfg_read32(register);

        self.cfg_write32(register, 0xffff_ffff);
        let ones = self.cfg_read32(register);

        self.cfg_write32(register, 0x0);
        let zeros = self.cfg_read32(register);

        self.cfg_write32(register, value);

        (value, ones ^ zeros)
    }

    fn moving_bits16(&self, register: u16) -> (u16, u16) {
        let value = self.cfg_read16(register);

        self.cfg_write16(register, 0xffff);
        let ones = self.cfg_read16(register);

        self.cfg_write16(register, 0x0);
        let zeros = self.cfg_read16(register);

        self.cfg_write16(register, value);

        (value, ones ^ zeros)
    }

    fn read_extended_allocation(&mut self, cap: PciCap) {
        let num_entries = self.cfg_read32(cap.base_register) >> 16 & 0x3f;
        let mut entry_base = if self.is_bridge() {
            // For Type 1 functions there is a second DW in the
            // capability, preceding the first entry. The second DW
            // includes fixed Secondary and Subordinate bus numbers,
            // if there are EA-capable functions behind the bridge,
            // otherwise 0s.
            cap.base_register + 8
        } else {
            cap.base_register + 4
        };

        for _ in 0..num_entries {
            let base_lo = self.cfg_read32(entry_base + 4);
            let max_offset_lo = self.cfg_read32(entry_base + 8);

            let is_64b = base_lo & 0x2 == 0x2;
            let (base, max_offset) = if is_64b {
                let base_hi = self.cfg_read32(entry_base + 12);
                let max_offset_hi = self.cfg_read32(entry_base + 16);
                (
                    (base_lo & !3) as u64 | (base_hi as u64) << 32,
                    (max_offset_lo | 3) as u64 | (max_offset_hi as u64) << 32,
                )
            } else {
                ((base_lo & !3) as u64, (max_offset_lo | 3) as u64)
            };

            let entry_info = self.cfg_read32(entry_base);
            let entry_size = entry_info & 0x7;
            // BAR entry index
            let bei = entry_info >> 4 & 0xf;
            let enabled = entry_info >> 31;
            let props = entry_info >> 8 & 0xff;
            let ty = if props == 2 {
                ResourceType::Io
            } else if is_64b {
                ResourceType::Memory64
            } else {
                ResourceType::Memory32
            };

            entry_base += (entry_size * 4) as u16;

            // Only add BARs 0-5
            if bei <= 5 && enabled == 1 {
                let size = max_offset + 1;
                if size > 0 {
                    self.resources[bei as usize] = Some(Resource {
                        bdf: self.bdf,
                        idx: BARS[bei as usize],
                        ty,
                        base: Some(base),
                        size,
                        alignment: size,
                        limit: base + max_offset,
                        prefetchable: props == 1,
                        bridge: false,
                    });
                }
            }
        }
    }

    // Bridges have extra "resources" which are ranges of address space
    // decoded to the downstream buses; one for non-prefetchable areas,
    // which only has 32 bits of space, and one which is prefetchable,
    // which has 64 bits of space
    fn read_bridge_resources(&mut self) {
        // Check for a non-prefetchable memory region
        if let Some((alignment, limit)) = {
            let (_, moving_base) = self.moving_bits16(Register::MemoryBase as u16);
            let (_, moving_limit) = self.moving_bits16(Register::MemoryLimit as u16);
            let moving = (moving_base as u64) << 16 & (moving_limit as u64) << 16;
            if moving != 0 {
                let mut step = 1;
                let mut align = 0;
                while (moving & step) == 0 {
                    step <<= 1;
                    align += 1;
                }

                Some((align, moving | (step - 1)))
            } else {
                None
            }
        } {
            self.resources[2] = Some(Resource {
                bdf: self.bdf,
                idx: Register::MemoryBase,
                ty: ResourceType::Memory32,
                base: None,
                size: 0,
                alignment,
                limit,
                prefetchable: false,
                bridge: true,
            });
        }

        // Check for a prefetchable memory region
        if let Some((alignment, limit)) = {
            let (_, moving_base_lower) = self.moving_bits16(Register::PrefMemBase as u16);
            let (_, moving_base_upper) = self.moving_bits32(Register::PrefBaseUpper as u16);
            let moving_base = (moving_base_lower as u64) << 16 | (moving_base_upper as u64) << 32;

            let (_, moving_limit_lower) = self.moving_bits16(Register::PrefMemLimit as u16);
            let (_, moving_limit_upper) = self.moving_bits32(Register::PrefLimitUpper as u16);
            let moving_limit =
                (moving_limit_lower as u64) << 16 | (moving_limit_upper as u64) << 32;

            let moving = moving_base & moving_limit;
            if moving != 0 {
                let mut step = 1;
                let mut align = 0;
                while (moving & step) == 0 {
                    step <<= 1;
                    align += 1;
                }

                Some((align, moving | (step - 1)))
            } else {
                None
            }
        } {
            self.resources[3] = Some(Resource {
                bdf: self.bdf,
                idx: Register::PrefMemBase,
                ty: ResourceType::Memory64,
                base: None,
                size: 0,
                alignment,
                limit,
                prefetchable: true,
                bridge: true,
            });
        }
    }

    fn size_bar(&self, register: Register) -> Option<Resource> {
        let (moving, attr) = {
            let (value, moving) = self.moving_bits32(register as u16);
            let (value, moving) = (value as u64, moving as u64);
            let attr = value & BAR_ATTRIBUTES_MASK;

            // Found 64-bit Memory BAR, read the next one and combine them
            if (attr & BASE_ADDRESS_MASK == BASE_ADDRESS_MEM)
                && (attr & MEM_LIMIT_MASK == MEM_LIMIT_64)
            {
                let (_, m) = self.moving_bits32(register as u16 + 4);
                (moving | ((m as u64) << 32), attr)
            } else {
                (moving, attr)
            }
        };

        if moving == 0 {
            return None;
        }

        let (size, align) = {
            let mut size = 1;
            let mut align = 0;
            while (moving & size) == 0 {
                size <<= 1;
                align += 1;
            }
            (size, align)
        };

        let is_mem = attr & BASE_ADDRESS_MASK == BASE_ADDRESS_MEM;
        let is_pref = is_mem && attr & BAR_PREFETCHABLE == BAR_PREFETCHABLE;

        let limit = if is_mem {
            let limit = match attr & MEM_LIMIT_MASK {
                MEM_LIMIT_32 => 0xffff_ffff,
                MEM_LIMIT_1M => 0xf_ffff,
                MEM_LIMIT_64 => 0xffff_ffff_ffff_ffff,
                _ => moving | (size - 1),
            };

            // Ensure the limit doesn't exceed the bits that move, though
            u64::min(limit, moving | (size - 1))
        } else {
            0xffff
        };

        // Ensure minimum alignment for memory resources is a page
        // TODO: this logic should probably go in the resource
        // allocator.
        //if is_mem && align < 12 {
        //align = 12;
        //}

        let ty = if is_mem {
            if attr & MEM_LIMIT_MASK == MEM_LIMIT_64 {
                ResourceType::Memory64
            } else {
                ResourceType::Memory32
            }
        } else {
            ResourceType::Io
        };

        Some(Resource {
            bdf: self.bdf,
            idx: register,
            ty,
            base: None,
            size,
            alignment: align,
            limit,
            prefetchable: is_pref,
            bridge: false,
        })
    }

    fn probe(bdf: Bdf, ops: &'a PciEcamCfgOps) -> Option<Self> {
        let device_id = ops.read16(&bdf, Register::DeviceId as u16);
        let vendor_id = ops.read16(&bdf, Register::VendorId as u16);

        if device_id == u16::MAX || vendor_id == u16::MAX {
            return None;
        }

        let hdr_type = ops.read8(&bdf, Register::HeaderType as u16) & !MULTI_FUNCTION;
        let header: HeaderType = match hdr_type.try_into() {
            Ok(h) => h,
            Err(_) => return None,
        };
        let device_info = DeviceInfo::new(
            ops.read8(&bdf, CLASS_CODE_OFFSET),
            ops.read8(&bdf, SUBCLASS_OFFSET),
            ops.read8(&bdf, PROG_IF_OFFSET),
        );

        let revision = ops.read8(&bdf, Register::RevisionId as u16);

        Some(Self {
            ops,
            bdf,
            vendor_id,
            device_id,
            header,
            device_info,
            revision,
            resources: [None, None, None, None, None, None],
        })
    }

    pub fn is_bridge(&self) -> bool {
        self.header == HeaderType::Bridge
    }
}

pub struct DeviceIterator<'a> {
    bdf: Bdf,
    ops: &'a PciEcamCfgOps,
    max_bus: Bus,
    done: bool,
}

impl<'a> DeviceIterator<'a> {
    fn new(bdf: Bdf, ops: &'a PciEcamCfgOps, max_bus: Bus) -> Self {
        Self {
            bdf,
            ops,
            max_bus,
            done: false,
        }
    }
}

impl<'a> core::iter::Iterator for DeviceIterator<'a> {
    type Item = Device<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            if self.done || self.bdf.bus >= self.max_bus {
                return None;
            }

            let dev = Device::probe(self.bdf, self.ops);

            // Check for multi-function device if this is function 0
            let mut skipped = false;
            if let Some(dev) = dev {
                if dev.bdf.func == Func(0) {
                    let reg = self.ops.read8(&dev.bdf, HEADER_TYPE_OFFSET);
                    if reg & MULTI_FUNCTION == 0 {
                        if let Some(b) = self.bdf.next_dev() {
                            self.bdf = b;
                            skipped = true;
                        } else {
                            self.done = true;
                        }
                    }
                }
            }

            if !skipped {
                if let Some(b) = self.bdf.next() {
                    self.bdf = b;
                } else {
                    self.done = true;
                }
            }

            if dev.is_some() {
                return dev;
            }
        }
    }
}

fn bridge_route(ops: &PciEcamCfgOps, this_bus: u8, next_bus: u8, dev: &Device) -> u8 {
    ops.write8(&dev.bdf, Register::PrimaryBus as u16, this_bus);
    ops.write8(&dev.bdf, Register::SecondaryBus as u16, next_bus);

    // Temporarily initialize subordinate bus to the maximum bus number.
    // This ensure that PCI configuration transactions will pass through
    // any PCI-PCI bridges
    ops.write8(&dev.bdf, Register::SubordinateBus as u16, MAX_BUS.0);

    // Recursively scan the new bus and absorb its highest sub bus number
    let new_primary = next_bus;
    let new_secondary = next_bus + 1;
    let subordinate = bridge_hierarchy(new_primary, ops, new_secondary);

    // Now set the new max bus number for this bridge
    ops.write8(&dev.bdf, Register::SubordinateBus as u16, subordinate);

    subordinate
}

fn close_bridge(ops: &PciEcamCfgOps, dev: &Device) {
    // Closing off of the hierarchy must be done in reverse order,
    // so it is a depth-first traversal that closes off bridges on
    // the way back down the tree.

    let secondary = ops.read8(&dev.bdf, Register::SecondaryBus as u16);
    close_hierarchy(secondary, ops);

    ops.write8(&dev.bdf, Register::PrimaryBus as u16, 0);
    ops.write8(&dev.bdf, Register::SecondaryBus as u16, 0xff);
    ops.write8(&dev.bdf, Register::SubordinateBus as u16, 0xfe);
}

// Returns the subordinate bus number
pub fn bridge_hierarchy(bus: u8, ops: &PciEcamCfgOps, next_bus: u8) -> u8 {
    let mut highest_bus = bus;
    let mut next_bus = next_bus;

    for device in all_local_devices(bus, ops) {
        if device.is_bridge() {
            let highest_downstream = bridge_route(ops, bus, next_bus, &device);
            highest_bus = u8::max(highest_bus, highest_downstream);
            next_bus = highest_bus + 1;
        }
    }

    highest_bus
}

// Closes a PCIe hierarchy from being enumerated
pub fn close_hierarchy(bus: u8, ops: &PciEcamCfgOps) {
    let end_bus = MAX_BUS;
    let start_bus = Bdf::from_bus(Bus(bus));

    for device in DeviceIterator::new(start_bus, ops, end_bus) {
        if device.is_bridge() {
            close_bridge(ops, &device);
        }
    }
}

// Scan only the provided `bus` for a device with the matching `vendor_id` and `device_id`
pub fn shallow_scan_for_device(
    bus: u8,
    ops: &PciEcamCfgOps,
    vendor_id: u16,
    device_id: u16,
) -> Option<Device> {
    let start_bus = Bdf::from_bus(Bus(bus));
    let end_bus = Bus(bus + 1);

    DeviceIterator::new(start_bus, ops, end_bus)
        .find(|&device| device.vendor_id == vendor_id && device.device_id == device_id)
}

// Scan as many devices as can be discovered starting at the provided `bus`.
pub fn all_devices(bus: u8, ops: &PciEcamCfgOps, max_bus: Bus) -> DeviceIterator {
    DeviceIterator::new(Bdf::from_bus(Bus(bus)), ops, max_bus)
}

/// Scan only the devices attached directly to the provided `bus`.
pub fn all_local_devices(bus: u8, ops: &PciEcamCfgOps) -> DeviceIterator {
    DeviceIterator::new(Bdf::from_bus(Bus(bus)), ops, Bus(bus + 1))
}

#[derive(Copy, Clone)]
pub struct Device<'a> {
    ops: &'a PciEcamCfgOps,
    pub bdf: Bdf,
    pub resources: [Option<Resource>; 6],
    pub header: HeaderType,
    pub vendor_id: u16,
    pub device_id: u16,
    pub device_info: DeviceInfo,
    pub revision: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bdf() {
        let b = Bdf::from_bus(Bus(0));
        assert_eq!(b.next(), Some(Bdf::new(Bus(0), Dev(0), Func(1))));
        let b = Bdf::new(Bus(0), Dev(0), Func(1));
        assert_eq!(b.next(), Some(Bdf::new(Bus(0), Dev(0), Func(2))));

        let b = Bdf::new(Bus(0), Dev(0), Func(7));
        assert_eq!(b.next(), Some(Bdf::new(Bus(0), Dev(1), Func(0))));

        let b = Bdf::new(Bus(0), Dev(1), Func(7));
        assert_eq!(b.next(), Some(Bdf::new(Bus(0), Dev(2), Func(0))));

        let b = Bdf::new(Bus(0), Dev(31), Func(7));
        assert_eq!(b.next(), Some(Bdf::new(Bus(1), Dev(0), Func(0))));

        let b = Bdf::new(Bus(1), Dev(0), Func(0));
        assert_eq!(b.next(), Some(Bdf::new(Bus(1), Dev(0), Func(1))));

        let b = Bdf::new(Bus(255), Dev(31), Func(7));
        assert_eq!(b.next(), None);
    }
}
