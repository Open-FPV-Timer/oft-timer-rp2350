//! A network device that only sends while the cable is in.
//!
//! The network stack keeps producing frames with the link down (DHCP discovers, retries).
//! The W6300 cannot send them, so they pile up in its transmit buffer, and once that is
//! full the driver waits for room that never comes; from then on it no longer notices the
//! cable going back in. Seen on hardware: powered up without a cable, plugged in minutes
//! later, no address until a restart.
//!
//! With the link down this wrapper tells the stack "nothing can be sent now". Asking for the
//! link state registers the stack's waker, so it tries again as soon as the link changes.

use core::task::Context;
use embassy_net::driver::{Capabilities, Driver, HardwareAddress, LinkState};

pub struct LinkGated<D>(pub D);

impl<D: Driver> Driver for LinkGated<D> {
    type RxToken<'a>
        = D::RxToken<'a>
    where
        Self: 'a;
    type TxToken<'a>
        = D::TxToken<'a>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // The receive token comes with a transmit token for the reply, so gate it too.
        // Nothing arrives without a link anyway.
        if self.0.link_state(cx) == LinkState::Down {
            return None;
        }
        self.0.receive(cx)
    }

    fn transmit(&mut self, cx: &mut Context) -> Option<Self::TxToken<'_>> {
        if self.0.link_state(cx) == LinkState::Down {
            return None;
        }
        self.0.transmit(cx)
    }

    fn link_state(&mut self, cx: &mut Context) -> LinkState {
        self.0.link_state(cx)
    }

    fn capabilities(&self) -> Capabilities {
        self.0.capabilities()
    }

    fn hardware_address(&self) -> HardwareAddress {
        self.0.hardware_address()
    }
}
