package rfm69

import (
	"log"
	"time"
)

const (
	debug         = false
	maxPacketSize = 110
	fifoSize      = 66

	// The fifoThreshold value should allow a maximum-sized packet to be
	// written in two bursts, but be large enough to avoid fifo underflow.
	fifoThreshold = 20

	// Approximate time for one byte to be transmitted, based on the data rate.
	byteDuration = time.Millisecond
)

func init() {
	if debug {
		log.SetFlags(log.Ltime | log.Lmicroseconds | log.LUTC)
	}
}

// Send transmits the given packet.
// The RadioHead protocol uses the first five bytes of the payload as a header:
// LENGTH, TO, FROM, ID, FLAGS. The LENGTH value is inclusive of the last four header
// bytes but exclusive of the LENGTH byte (so the actual LENGTH is payload + 4).
func (r *Radio) SendRadioHead(data []byte, to byte, from byte, id byte, flags byte) {
	if r.Error() != nil {
		return
	}
	if len(data) > maxPacketSize {
		log.Panicf("attempting to send %d-byte packet", len(data))
	}
	if debug {
		log.Printf("sending %d-byte packet in %s state", len(data), r.State())
	}
	// Add RadioHead headers
	r.txPacket[0] = byte(len(data)) + 4
	r.txPacket[1] = to
	r.txPacket[2] = from
	r.txPacket[3] = id
	r.txPacket[4] = flags
	copy(r.txPacket[5:], data)
	r.setMode(StandbyMode)
	r.transmit(r.txPacket[:byte(len(data))+5])
	r.setMode(StandbyMode)
}

// keep the standard Radio interface by implementing a Send() method
// with the default RadioHead header values (0xFF = broadcast)
func (r *Radio) Send(data []byte) {
	r.SendRadioHead(data, 0xFF, 0xFF, 0x00, 0x00)
}

func (r *Radio) transmit(data []byte) {
	if len(data) > fifoSize {
		log.Panicf("Send packet too big, %d bytes!", len(data))
	}
	r.clearFIFO()
	r.hw.WriteRegister(RegAutoModes, 0)
	if debug {
		log.Print("fifo clear, writing to fifo:")
		for _, b := range data {
			log.Printf("0x%02X", b)
		}
	}
	r.hw.WriteBurst(RegFifo, data)
	r.setMode(TransmitterMode)
	for r.Error() == nil {
		if (r.hw.ReadRegister(RegIrqFlags2) & 0x08) != 0 {
			break
		}
		//log.Print("Transmit not done yet")
		time.Sleep(byteDuration)
	}
	log.Print("Transmit done!")
}

func (r *Radio) fifoEmpty() bool {
	return r.hw.ReadRegister(RegIrqFlags2)&FifoNotEmpty == 0
}

func (r *Radio) fifoFull() bool {
	return r.hw.ReadRegister(RegIrqFlags2)&FifoFull != 0
}

func (r *Radio) fifoThresholdExceeded() bool {
	return r.hw.ReadRegister(RegIrqFlags2)&FifoLevel != 0
}

func (r *Radio) clearFIFO() {
	r.hw.WriteRegister(RegIrqFlags2, FifoOverrun)
}

// Receive listens with the given timeout for an incoming packet.
// It returns the packet and the associated RSSI.
// The RadioHead protocol uses the first five bytes of the payload as a header:
// LENGTH, TO, FROM, ID, FLAGS. The LENGTH value is inclusive of the last four header
// bytes but exclusive of the LENGTH byte (so the actual LENGTH is payload + 4).
// The packet is returned from this function with the four header bytes at the
// head, so the caller can read and/or discard them.
func (r *Radio) Receive(timeout time.Duration) ([]byte, int) {
	if r.Error() != nil {
		return nil, 0
	}
	r.hw.WriteRegister(RegAutoModes, 0)
	r.setMode(ReceiverMode)
	defer r.setMode(SleepMode)
	if debug {
		log.Printf("waiting for interrupt in %s state", r.State())
	}
	r.hw.AwaitInterrupt(timeout)
	rssi := r.ReadRSSI()
	length := -1
	for r.Error() == nil {
		if r.fifoEmpty() {
			if timeout <= 0 {
				break
			}
			time.Sleep(byteDuration)
			timeout -= byteDuration
			continue
		}
		c := r.hw.ReadRegister(RegFifo)
		if r.Error() != nil {
			break
		}
		if length == -1 {
			length = int(c)
		} else {
			r.err = r.receiveBuffer.WriteByte(c)
			if r.receiveBuffer.Len() == length {
				// End of packet.
				return r.finishRX(rssi)
			}
		}
	}
	return nil, rssi
}

func (r *Radio) finishRX(rssi int) ([]byte, int) {
	r.setMode(StandbyMode)
	size := r.receiveBuffer.Len()
	if size == 0 {
		return nil, rssi
	}
	p := make([]byte, size)
	_, r.err = r.receiveBuffer.Read(p)
	if r.Error() != nil {
		return nil, rssi
	}
	r.receiveBuffer.Reset()
	if debug {
		log.Printf("received %d-byte packet in %s state", size, r.State())
	}
	return p, rssi
}

// SendAndReceive transmits the given packet,
// then listens with the given timeout for an incoming packet.
// It returns the packet and the associated RSSI.
// (This could be further optimized by using an Automode to go directly
// from TX to RX, rather than returning to standby in between.)
func (r *Radio) SendAndReceive(data []byte, timeout time.Duration) ([]byte, int) {
	r.Send(data)
	if r.Error() != nil {
		return nil, 0
	}
	return r.Receive(timeout)
}
