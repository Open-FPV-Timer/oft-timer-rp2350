//! A small mDNS / DNS-SD responder for one service on one IPv4 address: enough for a client
//! to browse `_oft._tcp.local.`, resolve the Timer's instance to a host, port and TXT record,
//! and look up `<host>.local`. Message building and parsing only; no sockets here.
//!
//! What it answers (RFC 6762, RFC 6763), for a Timer `oft-a867`:
//!
//! ```text
//! _services._dns-sd._udp.local  PTR  ->  _oft._tcp.local
//! _oft._tcp.local               PTR  ->  oft-a867._oft._tcp.local    (+ SRV, TXT, A)
//! oft-a867._oft._tcp.local      SRV  ->  oft-a867.local : port       (+ A)
//! oft-a867._oft._tcp.local      TXT  ->  id=.. fw=.. proto=.. seats=.. nodes=.. level=..
//! oft-a867.local                A    ->  the address
//! ```
//!
//! Kept simple on purpose: names are written out in full (no compression in what it sends,
//! though it reads compressed questions), there is no probing for name conflicts (the name
//! comes from the chip's unique id) and no known-answer suppression (it may answer a
//! question the asker already knew the answer to; harmless).

pub const PORT: u16 = 5353;
pub const GROUP: [u8; 4] = [224, 0, 0, 251];

const TYPE_A: u16 = 1;
const TYPE_PTR: u16 = 12;
const TYPE_TXT: u16 = 16;
const TYPE_SRV: u16 = 33;
const TYPE_ANY: u16 = 255;
const CLASS_IN: u16 = 1;
/// In a question: "a unicast reply is fine". In a record: "this replaces what you cached".
const TOP_BIT: u16 = 0x8000;

/// Lifetimes, seconds. Short for what follows the address, long for the rest (RFC 6762 §10).
const TTL_HOST: u32 = 120;
const TTL_OTHER: u32 = 4500;
/// A reply to a one-shot resolver must not be cached for long (RFC 6762 §6.7).
const TTL_LEGACY: u32 = 10;

/// What is advertised.
#[derive(Clone, Copy)]
pub struct Service<'a> {
    /// Service type without the domain, e.g. `_oft._tcp`.
    pub service: &'a str,
    /// Instance name (one label), e.g. `oft-a867`.
    pub instance: &'a str,
    /// Host name (one label), e.g. `oft-a867`; advertised as `<host>.local`.
    pub host: &'a str,
    pub port: u16,
    pub addr: [u8; 4],
    /// TXT pairs, written as `key=value`.
    pub txt: &'a [(&'a str, &'a str)],
}

/// Which of the Timer's names a question asked about.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Name {
    /// `_services._dns-sd._udp.local`
    Enumerate,
    /// `<service>.local`
    Type,
    /// `<instance>.<service>.local`
    Instance,
    /// `<host>.local`
    Host,
}

struct Writer<'a> {
    buf: &'a mut [u8],
    len: usize,
    ok: bool,
}

impl<'a> Writer<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, len: 0, ok: true }
    }

    fn put(&mut self, bytes: &[u8]) {
        if self.len + bytes.len() > self.buf.len() {
            self.ok = false;
            return;
        }
        self.buf[self.len..self.len + bytes.len()].copy_from_slice(bytes);
        self.len += bytes.len();
    }

    fn u16(&mut self, v: u16) {
        self.put(&v.to_be_bytes());
    }

    fn u32(&mut self, v: u32) {
        self.put(&v.to_be_bytes());
    }

    /// A dotted name as DNS labels, ending with the root label.
    fn name(&mut self, parts: &[&str]) {
        for part in parts {
            for label in part.split('.').filter(|l| !l.is_empty()) {
                let l = label.as_bytes();
                if l.len() > 63 {
                    self.ok = false;
                    return;
                }
                self.put(&[l.len() as u8]);
                self.put(l);
            }
        }
        self.put(&[0]);
    }

    /// Record header up to the data length, which `end_record` fills in.
    fn record(&mut self, owner: &[&str], rtype: u16, class: u16, ttl: u32) -> usize {
        self.name(owner);
        self.u16(rtype);
        self.u16(class);
        self.u32(ttl);
        self.u16(0);
        self.len
    }

    fn end_record(&mut self, data_start: usize) {
        if self.ok {
            let n = (self.len - data_start) as u16;
            self.buf[data_start - 2..data_start].copy_from_slice(&n.to_be_bytes());
        }
    }
}

/// Which records go into a reply.
#[derive(Clone, Copy, Default, PartialEq, Eq, Debug)]
struct Wanted {
    enumerate: bool,
    ptr: bool,
    srv: bool,
    txt: bool,
    a: bool,
}

impl Wanted {
    fn any(self) -> bool {
        self.enumerate || self.ptr || self.srv || self.txt || self.a
    }

    fn count(self) -> u16 {
        self.enumerate as u16 + self.ptr as u16 + self.srv as u16 + self.txt as u16 + self.a as u16
    }
}

fn records(w: &mut Writer, s: &Service, want: Wanted, legacy: bool) {
    let ttl = |t: u32| if legacy { TTL_LEGACY } else { t };
    // Records that belong to this Timer alone carry the "replaces your cache" bit; the
    // shared PTR records do not. One-shot resolvers must not see the bit at all.
    let unique = if legacy { CLASS_IN } else { CLASS_IN | TOP_BIT };
    if want.enumerate {
        let at = w.record(&["_services._dns-sd._udp", "local"], TYPE_PTR, CLASS_IN, ttl(TTL_OTHER));
        w.name(&[s.service, "local"]);
        w.end_record(at);
    }
    if want.ptr {
        let at = w.record(&[s.service, "local"], TYPE_PTR, CLASS_IN, ttl(TTL_OTHER));
        w.name(&[s.instance, s.service, "local"]);
        w.end_record(at);
    }
    if want.srv {
        let at = w.record(&[s.instance, s.service, "local"], TYPE_SRV, unique, ttl(TTL_HOST));
        w.u16(0);
        w.u16(0);
        w.u16(s.port);
        w.name(&[s.host, "local"]);
        w.end_record(at);
    }
    if want.txt {
        let at = w.record(&[s.instance, s.service, "local"], TYPE_TXT, unique, ttl(TTL_OTHER));
        if s.txt.is_empty() {
            w.put(&[0]);
        }
        for (k, v) in s.txt {
            let n = k.len() + 1 + v.len();
            if n > 255 {
                w.ok = false;
                return;
            }
            w.put(&[n as u8]);
            w.put(k.as_bytes());
            w.put(b"=");
            w.put(v.as_bytes());
        }
        w.end_record(at);
    }
    if want.a {
        let at = w.record(&[s.host, "local"], TYPE_A, unique, ttl(TTL_HOST));
        w.put(&s.addr);
        w.end_record(at);
    }
}

/// The unsolicited announcement: everything about the service, sent to the group when the
/// Timer comes up or something in it changes. Returns the length, or 0 if `out` is too small.
pub fn announce(s: &Service, out: &mut [u8]) -> usize {
    let want = Wanted { enumerate: false, ptr: true, srv: true, txt: true, a: true };
    let mut w = Writer::new(out);
    w.u16(0);
    w.u16(0x8400); // response, authoritative
    w.u16(0);
    w.u16(want.count());
    w.u16(0);
    w.u16(0);
    records(&mut w, s, want, false);
    if w.ok {
        w.len
    } else {
        0
    }
}

/// Read a name at `at`, following compression pointers, and compare it label by label
/// (ASCII case-insensitively) with `want`, a list of dotted parts. Returns the offset just
/// past the name in the message, and whether it matched.
fn read_name(msg: &[u8], mut at: usize, want: &[&str]) -> Option<(usize, bool)> {
    let mut labels = want.iter().flat_map(|p| p.split('.')).filter(|l| !l.is_empty());
    let mut matched = true;
    let mut end = None;
    for _ in 0..32 {
        let len = *msg.get(at)? as usize;
        if len == 0 {
            if labels.next().is_some() {
                matched = false;
            }
            return Some((end.unwrap_or(at + 1), matched));
        }
        if len & 0xc0 == 0xc0 {
            let to = ((len & 0x3f) << 8) | *msg.get(at + 1)? as usize;
            if end.is_none() {
                end = Some(at + 2);
            }
            // Pointers only ever point backwards; anything else is a malformed message.
            if to >= at {
                return None;
            }
            at = to;
            continue;
        }
        if len > 63 {
            return None;
        }
        let label = msg.get(at + 1..at + 1 + len)?;
        match labels.next() {
            Some(w) if w.as_bytes().eq_ignore_ascii_case(label) => {}
            _ => matched = false,
        }
        at += 1 + len;
    }
    None
}

/// What to do with a received message.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Reply {
    pub len: usize,
    /// Send to the asker's address and port instead of the group.
    pub unicast: bool,
}

/// Build the reply to a received mDNS message, if it asks about anything of ours.
/// `from_port` is the UDP source port: a query from any port but 5353 comes from a one-shot
/// resolver, which gets a conventional DNS reply (its id and question echoed, unicast).
pub fn respond(msg: &[u8], from_port: u16, s: &Service, out: &mut [u8]) -> Option<Reply> {
    if msg.len() < 12 {
        return None;
    }
    let flags = u16::from_be_bytes([msg[2], msg[3]]);
    // Only standard queries: not responses, opcode 0.
    if flags & 0xf800 != 0 {
        return None;
    }
    let questions = u16::from_be_bytes([msg[4], msg[5]]) as usize;
    let legacy = from_port != PORT;
    let mut want = Wanted::default();
    let mut extra = Wanted::default();
    let mut unicast_ok = questions > 0;
    // For a one-shot resolver: where its (single) question sits, to echo it back.
    let mut first_question = None;
    let mut at = 12;
    for q in 0..questions.min(16) {
        let start = at;
        let mut which = None;
        let mut end = at;
        for (name, parts) in [
            (Name::Enumerate, &["_services._dns-sd._udp", "local"][..]),
            (Name::Type, &[s.service, "local"][..]),
            (Name::Instance, &[s.instance, s.service, "local"][..]),
            (Name::Host, &[s.host, "local"][..]),
        ] {
            let (e, matched) = read_name(msg, at, parts)?;
            end = e;
            if matched {
                which = Some(name);
                break;
            }
        }
        let qtype = u16::from_be_bytes([*msg.get(end)?, *msg.get(end + 1)?]);
        let qclass = u16::from_be_bytes([*msg.get(end + 2)?, *msg.get(end + 3)?]);
        at = end + 4;
        if q == 0 {
            first_question = Some((start, at));
        }
        if qclass & !TOP_BIT != CLASS_IN && qclass & !TOP_BIT != 255 {
            continue;
        }
        let asks = |t: u16| qtype == t || qtype == TYPE_ANY;
        let mut hit = false;
        match which {
            Some(Name::Enumerate) if asks(TYPE_PTR) => {
                want.enumerate = true;
                hit = true;
            }
            Some(Name::Type) if asks(TYPE_PTR) => {
                want.ptr = true;
                extra.srv = true;
                extra.txt = true;
                extra.a = true;
                hit = true;
            }
            Some(Name::Instance) => {
                if asks(TYPE_SRV) {
                    want.srv = true;
                    extra.a = true;
                    hit = true;
                }
                if asks(TYPE_TXT) {
                    want.txt = true;
                    hit = true;
                }
            }
            Some(Name::Host) if asks(TYPE_A) => {
                want.a = true;
                hit = true;
            }
            _ => {}
        }
        if hit && qclass & TOP_BIT == 0 {
            unicast_ok = false;
        }
    }
    if !want.any() {
        return None;
    }
    // What is already an answer is not repeated as an extra.
    extra.srv &= !want.srv;
    extra.txt &= !want.txt;
    extra.a &= !want.a;
    extra.ptr = false;
    extra.enumerate = false;

    let mut w = Writer::new(out);
    let echo = if legacy { first_question } else { None };
    w.put(if legacy { &msg[0..2] } else { &[0, 0] });
    w.u16(0x8400);
    w.u16(echo.is_some() as u16);
    w.u16(want.count());
    w.u16(0);
    w.u16(extra.count());
    if let Some((a, b)) = echo {
        // The question is copied as it came. If its name used compression it can only point
        // at the header-relative offsets of the original, which a single question never does.
        w.put(msg.get(a..b)?);
    }
    records(&mut w, s, want, legacy);
    records(&mut w, s, extra, legacy);
    w.ok.then_some(Reply { len: w.len, unicast: legacy || unicast_ok })
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::string::String;
    use std::vec::Vec;

    const TXT: &[(&str, &str)] = &[("id", "oft-a867"), ("fw", "0.1.0"), ("proto", "1"), ("seats", "2"), ("nodes", "1"), ("level", "L2")];
    const SVC: Service = Service { service: "_oft._tcp", instance: "oft-a867", host: "oft-a867", port: 7270, addr: [192, 168, 36, 3], txt: TXT };

    fn query(id: u16, questions: &[(&str, u16, u16)]) -> Vec<u8> {
        let mut m = Vec::new();
        m.extend_from_slice(&id.to_be_bytes());
        m.extend_from_slice(&[0, 0]);
        m.extend_from_slice(&(questions.len() as u16).to_be_bytes());
        m.extend_from_slice(&[0; 6]);
        for (name, t, c) in questions {
            for l in name.split('.').filter(|l| !l.is_empty()) {
                m.push(l.len() as u8);
                m.extend_from_slice(l.as_bytes());
            }
            m.push(0);
            m.extend_from_slice(&t.to_be_bytes());
            m.extend_from_slice(&c.to_be_bytes());
        }
        m
    }

    /// A plain DNS parser for the tests: (owner, type, class, ttl, rdata) per record.
    #[derive(Debug)]
    struct Rec {
        owner: String,
        rtype: u16,
        class: u16,
        ttl: u32,
        data: Vec<u8>,
        data_at: usize,
    }

    fn name_at(m: &[u8], mut at: usize) -> (String, usize) {
        let mut out = String::new();
        let mut end = None;
        loop {
            let l = m[at] as usize;
            if l == 0 {
                return (out, end.unwrap_or(at + 1));
            }
            if l & 0xc0 == 0xc0 {
                end.get_or_insert(at + 2);
                at = ((l & 0x3f) << 8) | m[at + 1] as usize;
                continue;
            }
            out.push_str(core::str::from_utf8(&m[at + 1..at + 1 + l]).unwrap());
            out.push('.');
            at += 1 + l;
        }
    }

    fn parse(m: &[u8]) -> (u16, u16, [u16; 4], Vec<Rec>) {
        let h = |i: usize| u16::from_be_bytes([m[i], m[i + 1]]);
        let counts = [h(4), h(6), h(8), h(10)];
        let mut at = 12;
        for _ in 0..counts[0] {
            at = name_at(m, at).1 + 4;
        }
        let mut recs = Vec::new();
        for _ in 0..counts[1] + counts[2] + counts[3] {
            let (owner, e) = name_at(m, at);
            let rtype = u16::from_be_bytes([m[e], m[e + 1]]);
            let class = u16::from_be_bytes([m[e + 2], m[e + 3]]);
            let ttl = u32::from_be_bytes([m[e + 4], m[e + 5], m[e + 6], m[e + 7]]);
            let n = u16::from_be_bytes([m[e + 8], m[e + 9]]) as usize;
            recs.push(Rec { owner, rtype, class, ttl, data: m[e + 10..e + 10 + n].to_vec(), data_at: e + 10 });
            at = e + 10 + n;
        }
        assert_eq!(at, m.len(), "trailing bytes or a wrong count");
        (h(0), h(2), counts, recs)
    }

    fn reply(q: &[u8], port: u16) -> Option<(Reply, Vec<u8>)> {
        let mut out = [0u8; 512];
        respond(q, port, &SVC, &mut out).map(|r| (r, out[..r.len].to_vec()))
    }

    #[test]
    fn browse_gets_everything_needed_to_connect() {
        let (r, m) = reply(&query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN)]), PORT).unwrap();
        assert!(!r.unicast);
        let (id, flags, counts, recs) = parse(&m);
        assert_eq!((id, flags, counts), (0, 0x8400, [0, 1, 0, 3]));
        assert_eq!((recs[0].owner.as_str(), recs[0].rtype, recs[0].class, recs[0].ttl), ("_oft._tcp.local.", TYPE_PTR, CLASS_IN, 4500));
        assert_eq!(name_at(&m, recs[0].data_at).0, "oft-a867._oft._tcp.local.");
        let srv = recs.iter().find(|r| r.rtype == TYPE_SRV).unwrap();
        assert_eq!((srv.owner.as_str(), srv.class, srv.ttl), ("oft-a867._oft._tcp.local.", CLASS_IN | TOP_BIT, 120));
        assert_eq!(&srv.data[..6], &[0, 0, 0, 0, 0x1c, 0x66]); // priority, weight, port 7270
        assert_eq!(name_at(&m, srv.data_at + 6).0, "oft-a867.local.");
        let txt = recs.iter().find(|r| r.rtype == TYPE_TXT).unwrap();
        let mut pairs = Vec::new();
        let mut i = 0;
        while i < txt.data.len() {
            let n = txt.data[i] as usize;
            pairs.push(String::from_utf8(txt.data[i + 1..i + 1 + n].to_vec()).unwrap());
            i += 1 + n;
        }
        assert_eq!(pairs, ["id=oft-a867", "fw=0.1.0", "proto=1", "seats=2", "nodes=1", "level=L2"]);
        let a = recs.iter().find(|r| r.rtype == TYPE_A).unwrap();
        assert_eq!((a.owner.as_str(), a.data.as_slice(), a.ttl), ("oft-a867.local.", &[192, 168, 36, 3][..], 120));
    }

    #[test]
    fn each_name_answers_its_own_question() {
        let (_, m) = reply(&query(0, &[("_services._dns-sd._udp.local", TYPE_PTR, CLASS_IN)]), PORT).unwrap();
        let (_, _, counts, recs) = parse(&m);
        assert_eq!(counts, [0, 1, 0, 0]);
        assert_eq!(name_at(&m, recs[0].data_at).0, "_oft._tcp.local.");

        let (_, m) = reply(&query(0, &[("oft-a867._oft._tcp.local", TYPE_SRV, CLASS_IN)]), PORT).unwrap();
        let (_, _, counts, recs) = parse(&m);
        assert_eq!(counts, [0, 1, 0, 1]);
        assert_eq!((recs[0].rtype, recs[1].rtype), (TYPE_SRV, TYPE_A));

        let (_, m) = reply(&query(0, &[("oft-a867._oft._tcp.local", TYPE_TXT, CLASS_IN)]), PORT).unwrap();
        assert_eq!(parse(&m).2, [0, 1, 0, 0]);

        let (_, m) = reply(&query(0, &[("OFT-A867.Local", TYPE_A, CLASS_IN)]), PORT).unwrap();
        let (_, _, counts, recs) = parse(&m);
        assert_eq!((counts, recs[0].data.as_slice()), ([0, 1, 0, 0], &[192, 168, 36, 3][..]));

        // ANY on the instance: SRV and TXT as answers, the address as an extra
        let (_, m) = reply(&query(0, &[("oft-a867._oft._tcp.local", TYPE_ANY, CLASS_IN)]), PORT).unwrap();
        assert_eq!(parse(&m).2, [0, 2, 0, 1]);

        // two questions in one message: nothing is sent twice
        let (_, m) = reply(&query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN), ("oft-a867.local", TYPE_A, CLASS_IN)]), PORT).unwrap();
        let (_, _, counts, recs) = parse(&m);
        assert_eq!(counts, [0, 2, 0, 2]);
        assert_eq!(recs.iter().filter(|r| r.rtype == TYPE_A).count(), 1);
    }

    #[test]
    fn silence_for_everything_else() {
        assert!(reply(&query(0, &[("_http._tcp.local", TYPE_PTR, CLASS_IN)]), PORT).is_none());
        assert!(reply(&query(0, &[("oft-beef.local", TYPE_A, CLASS_IN)]), PORT).is_none());
        assert!(reply(&query(0, &[("oft-a867.local", 28, CLASS_IN)]), PORT).is_none()); // AAAA
        assert!(reply(&query(0, &[("oft-a867.local.extra", TYPE_A, CLASS_IN)]), PORT).is_none());
        assert!(reply(&query(0, &[("local", TYPE_A, CLASS_IN)]), PORT).is_none());
        assert!(reply(&query(0, &[("_oft._tcp.local", TYPE_PTR, 3)]), PORT).is_none()); // class CH
        // a response (someone else's announcement) is never answered
        let mut other = [0u8; 512];
        let n = announce(&SVC, &mut other);
        assert!(reply(&other[..n], PORT).is_none());
        // garbage and truncation
        assert!(reply(&[0; 5], PORT).is_none());
        let q = query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN)]);
        for cut in 12..q.len() {
            assert!(reply(&q[..cut], PORT).is_none(), "cut at {cut}");
        }
        // a name that points at itself
        let mut looped = query(0, &[("x", TYPE_A, CLASS_IN)]);
        looped[12] = 0xc0;
        looped[13] = 12;
        assert!(reply(&looped, PORT).is_none());
    }

    #[test]
    fn compressed_question_names_are_read() {
        // Two questions; the second name is "oft-a867" + a pointer to "local" in the first.
        let mut m = query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN)]);
        m[5] = 2;
        let local_at = 12 + 5 + 5; // after "\x04_oft" and "\x04_tcp"
        assert_eq!(&m[local_at..local_at + 6], b"\x05local");
        m.extend_from_slice(b"\x08oft-a867");
        m.extend_from_slice(&[0xc0, local_at as u8]);
        m.extend_from_slice(&[0, TYPE_A as u8, 0, 1]);
        let (_, out) = reply(&m, PORT).unwrap();
        assert_eq!(parse(&out).2, [0, 2, 0, 2]);
    }

    #[test]
    fn unicast_when_asked_and_for_one_shot_resolvers() {
        // QU bit set on the only question: a unicast reply is allowed
        let (r, _) = reply(&query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN | TOP_BIT)]), PORT).unwrap();
        assert!(r.unicast);
        // a one-shot resolver (dig, a plain DNS library): id and question echoed, short TTLs,
        // no cache-flush bit
        let q = query(0x1234, &[("oft-a867.local", TYPE_A, CLASS_IN)]);
        let (r, m) = reply(&q, 40000).unwrap();
        assert!(r.unicast);
        let (id, _, counts, recs) = parse(&m);
        assert_eq!((id, counts), (0x1234, [1, 1, 0, 0]));
        assert_eq!(&m[12..12 + q.len() - 12], &q[12..]);
        assert_eq!((recs[0].class, recs[0].ttl), (CLASS_IN, 10));
    }

    #[test]
    fn announcement_carries_the_whole_service() {
        let mut out = [0u8; 512];
        let n = announce(&SVC, &mut out);
        let (id, flags, counts, recs) = parse(&out[..n]);
        assert_eq!((id, flags, counts), (0, 0x8400, [0, 4, 0, 0]));
        assert_eq!(recs.iter().map(|r| r.rtype).collect::<Vec<_>>(), [TYPE_PTR, TYPE_SRV, TYPE_TXT, TYPE_A]);
        assert!(n < 300, "{n} bytes");
        // too small a buffer is reported, not overrun
        assert_eq!(announce(&SVC, &mut [0u8; 64]), 0);
        assert!(respond(&query(0, &[("_oft._tcp.local", TYPE_PTR, CLASS_IN)]), PORT, &SVC, &mut [0u8; 64]).is_none());
    }
}
