//! Firmware update logic for the RP2350 Timer that does not touch hardware, so it can be
//! tested on the host (`cargo test --target <host triple>` in this directory).
//!
//! - [`layout`]: where the two firmware slots live in flash and how to tell which one is running.
//! - [`image`]: what a valid Timer firmware image looks like (the boot ROM's "block loop" with
//!   a SHA-256 over the whole image) and the checks that decide whether to accept one.
//! - [`settings`]: where saved settings live in flash and the power-cut-safe record format.
//! - [`receiver`]: the order in which an upload is written to the spare slot, chosen so that
//!   stopping at any point leaves the running firmware as the one the boot ROM picks.
//!
//! The boot ROM does the switching and the rollback (A/B partitions with "try before you
//! buy", RP2350 datasheet sections 5.1.7, 5.1.16 and 5.1.17). Nothing here is a bootloader.

#![cfg_attr(not(test), no_std)]

pub mod esprom;
pub mod image;
pub mod layout;
pub mod receiver;
pub mod settings;
