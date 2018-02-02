package i2c

import (
	"encoding/binary"

	"time"

	"math"

	"fmt"

	"github.com/pkg/errors"
	"gobot.io/x/gobot"
)

const APDS9960Address = 0x39

const (

	/* Gesture parameters */
	APDS9960_GESTURE_THRESHOLD_OUT = 10
	APDS9960_GESTURE_SENSITIVITY_1 = 50
	APDS9960_GESTURE_SENSITIVITY_2 = 20

	/* Error code for returned values */
	APDS9960_ERROR = 0xFF

	/* Acceptable device IDs */
	APDS9960_ID_1 = 0xAB
	APDS9960_ID_2 = 0x9C

	/* Misc parameters */
	APDS9960_FIFO_PAUSE_TIME = 30 // Wait period (ms) between FIFO reads

	/* APDS-9960 register addresses */
	APDS9960_ENABLE     = 0x80
	APDS9960_ATIME      = 0x81
	APDS9960_WTIME      = 0x83
	APDS9960_AILTL      = 0x84
	APDS9960_AILTH      = 0x85
	APDS9960_AIHTL      = 0x86
	APDS9960_AIHTH      = 0x87
	APDS9960_PILT       = 0x89
	APDS9960_PIHT       = 0x8B
	APDS9960_PERS       = 0x8C
	APDS9960_CONFIG1    = 0x8D
	APDS9960_PPULSE     = 0x8E
	APDS9960_CONTROL    = 0x8F
	APDS9960_CONFIG2    = 0x90
	APDS9960_ID         = 0x92
	APDS9960_STATUS     = 0x93
	APDS9960_CDATAL     = 0x94
	APDS9960_CDATAH     = 0x95
	APDS9960_RDATAL     = 0x96
	APDS9960_RDATAH     = 0x97
	APDS9960_GDATAL     = 0x98
	APDS9960_GDATAH     = 0x99
	APDS9960_BDATAL     = 0x9A
	APDS9960_BDATAH     = 0x9B
	APDS9960_PDATA      = 0x9C
	APDS9960_POFFSET_UR = 0x9D
	APDS9960_POFFSET_DL = 0x9E
	APDS9960_CONFIG3    = 0x9F
	APDS9960_GPENTH     = 0xA0
	APDS9960_GEXTH      = 0xA1
	APDS9960_GCONF1     = 0xA2
	APDS9960_GCONF2     = 0xA3
	APDS9960_GOFFSET_U  = 0xA4
	APDS9960_GOFFSET_D  = 0xA5
	APDS9960_GOFFSET_L  = 0xA7
	APDS9960_GOFFSET_R  = 0xA9
	APDS9960_GPULSE     = 0xA6
	APDS9960_GCONF3     = 0xAA
	APDS9960_GCONF4     = 0xAB
	APDS9960_GFLVL      = 0xAE
	APDS9960_GSTATUS    = 0xAF
	APDS9960_IFORCE     = 0xE4
	APDS9960_PICLEAR    = 0xE5
	APDS9960_CICLEAR    = 0xE6
	APDS9960_AICLEAR    = 0xE7
	APDS9960_GFIFO_U    = 0xFC
	APDS9960_GFIFO_D    = 0xFD
	APDS9960_GFIFO_L    = 0xFE
	APDS9960_GFIFO_R    = 0xFF

	/* Bit fields */
	APDS9960_PON    = 0x01 //00000001
	APDS9960_AEN    = 0x02 //00000010
	APDS9960_PEN    = 0x04 //00000100
	APDS9960_WEN    = 0x08 //00001000
	APDS9960_AIEN   = 0x10 //00010000
	APDS9960_PIEN   = 0x20 //00100000
	APDS9960_GEN    = 0x40 //01000000
	APDS9960_GVALID = 0x01 //00000001

	/* On/Off definitions */
	APDS9960_OFF = 0
	APDS9960_ON  = 1

	/* Acceptable parameters for setMode */
	APDS9960_POWER             = 0
	APDS9960_AMBIENT_LIGHT     = 1
	APDS9960_PROXIMITY         = 2
	APDS9960_WAIT              = 3
	APDS9960_AMBIENT_LIGHT_INT = 4
	APDS9960_PROXIMITY_INT     = 5
	APDS9960_GESTURE           = 6
	APDS9960_ALL               = 7

	/* LED Drive values */
	APDS9960_LED_DRIVE_100MA  = 0
	APDS9960_LED_DRIVE_50MA   = 1
	APDS9960_LED_DRIVE_25MA   = 2
	APDS9960_LED_DRIVE_12_5MA = 3

	/* Proximity Gain (PGAIN) values */
	APDS9960_PGAIN_1X = 0
	APDS9960_PGAIN_2X = 1
	APDS9960_PGAIN_4X = 2
	APDS9960_PGAIN_8X = 3

	/* ALS Gain (AGAIN) values */
	APDS9960_AGAIN_1X  = 0
	APDS9960_AGAIN_4X  = 1
	APDS9960_AGAIN_16X = 2
	APDS9960_AGAIN_64X = 3

	/* Gesture Gain (GGAIN) values */
	APDS9960_GGAIN_1X = 0
	APDS9960_GGAIN_2X = 1
	APDS9960_GGAIN_4X = 2
	APDS9960_GGAIN_8X = 3

	/* LED Boost values */
	APDS9960_LED_BOOST_100 = 0
	APDS9960_LED_BOOST_150 = 1
	APDS9960_LED_BOOST_200 = 2
	APDS9960_LED_BOOST_300 = 3

	/* Gesture wait time values */
	APDS9960_GWTIME_0MS    = 0
	APDS9960_GWTIME_2_8MS  = 1
	APDS9960_GWTIME_5_6MS  = 2
	APDS9960_GWTIME_8_4MS  = 3
	APDS9960_GWTIME_14_0MS = 4
	APDS9960_GWTIME_22_4MS = 5
	APDS9960_GWTIME_30_8MS = 6
	APDS9960_GWTIME_39_2MS = 7

	/* Direction definitions */
	APDS9960_DIR_NONE  = 0
	APDS9960_DIR_LEFT  = 1
	APDS9960_DIR_RIGHT = 2
	APDS9960_DIR_UP    = 3
	APDS9960_DIR_DOWN  = 4
	APDS9960_DIR_NEAR  = 5
	APDS9960_DIR_FAR   = 6
	APDS9960_DIR_ALL   = 7

	/* State definitions */
	APDS9960_NA_STATE   = 0
	APDS9960_NEAR_STATE = 1
	APDS9960_FAR_STATE  = 2
	APDS9960_ALL_STATE  = 3

	/* Default values */
	APDS9960_DEFAULT_ATIME          = 219  // 103ms
	APDS9960_DEFAULT_WTIME          = 246  // 27ms
	APDS9960_DEFAULT_PROX_PPULSE    = 0x87 // 16us, 8 pulses
	APDS9960_DEFAULT_GESTURE_PPULSE = 0x89 // 16us, 10 pulses
	APDS9960_DEFAULT_POFFSET_UR     = 0    // 0 offset
	APDS9960_DEFAULT_POFFSET_DL     = 0    // 0 offset
	APDS9960_DEFAULT_CONFIG1        = 0x60 // No 12x wait (WTIME) factor
	APDS9960_DEFAULT_LDRIVE         = APDS9960_LED_DRIVE_100MA
	APDS9960_DEFAULT_PGAIN          = APDS9960_PGAIN_4X
	APDS9960_DEFAULT_AGAIN          = APDS9960_AGAIN_4X
	APDS9960_DEFAULT_PILT           = 0      // Low proximity threshold
	APDS9960_DEFAULT_PIHT           = 50     // High proximity threshold
	APDS9960_DEFAULT_AILT           = 0xFFFF // Force interrupt for calibration
	APDS9960_DEFAULT_AIHT           = 0
	APDS9960_DEFAULT_PERS           = 0x11 // 2 consecutive prox or ALS for int.
	APDS9960_DEFAULT_CONFIG2        = 0x01 // No saturation interrupts or LED boost
	APDS9960_DEFAULT_CONFIG3        = 0    // Enable all photodiodes, no SAI
	APDS9960_DEFAULT_GPENTH         = 40   // Threshold for entering gesture mode
	APDS9960_DEFAULT_GEXTH          = 30   // Threshold for exiting gesture mode
	APDS9960_DEFAULT_GCONF1         = 0x40 // 4 gesture events for int., 1 for exit
	APDS9960_DEFAULT_GGAIN          = APDS9960_GGAIN_4X
	APDS9960_DEFAULT_GLDRIVE        = APDS9960_LED_DRIVE_100MA
	APDS9960_DEFAULT_GWTIME         = APDS9960_GWTIME_2_8MS
	APDS9960_DEFAULT_GOFFSET        = 0    // No offset scaling for gesture mode
	APDS9960_DEFAULT_GPULSE         = 0xC9 // 32us, 10 pulses
	APDS9960_DEFAULT_GCONF3         = 0    // All photodiodes active during gesture
	APDS9960_DEFAULT_GIEN           = 0    // Disable gesture interrupts

)

// APDS9960Driver is the gobot driver for the color, proximity and gesture sensor APDS9960
type APDS9960Driver struct {
	name       string
	connector  Connector
	connection Connection

	gestureData apds9960GestureData
	gestureAux  apds9960GestureAux

	Config
}

// Internal structure for gesture data
type apds9960GestureData struct {
	u             [32]uint8
	d             [32]uint8
	l             [32]uint8
	r             [32]uint8
	index         uint8
	totalGestures uint8
	inThreshold   uint8
	outThreshold  uint8
}

// Internal structure for auxiliary gesture data
type apds9960GestureAux struct {
	udDelta   int
	lrDelta   int
	udCount   int
	lrCount   int
	nearCount int
	farCount  int
	state     int
	motion    int
}

// NewAPDS9960Driver creates a new driver with specified i2c interface
// Params:
//		conn Connector - the Adaptor to use with this Driver
//
// Optional params:
//		i2c.WithBus(int):	bus to use with this driver
//		i2c.WithAddress(int):	address to use with this driver
//
func NewAPDS9960Driver(a Connector, options ...func(Config)) *APDS9960Driver {
	m := &APDS9960Driver{
		name:      gobot.DefaultName("APDS9960"),
		connector: a,

		Config: NewConfig(),
	}

	for _, option := range options {
		option(m)
	}

	// TODO: add commands for API
	return m
}

// Name returns the Name for the Driver
func (h *APDS9960Driver) Name() string { return h.name }

// SetName sets the Name for the Driver
func (h *APDS9960Driver) SetName(n string) { h.name = n }

// Connection returns the connection for the Driver
func (h *APDS9960Driver) Connection() gobot.Connection { return h.connector.(gobot.Connection) }

// Start initialized the apds9960
func (h *APDS9960Driver) Start() (err error) {
	bus := h.GetBusOrDefault(h.connector.GetDefaultBus())
	address := h.GetAddressOrDefault(APDS9960Address)

	h.connection, err = h.connector.GetConnection(address, bus)
	if err != nil {
		return err
	}

	// Read ID and check it's a APDS9960
	var id uint8
	id, err = h.connection.ReadByteData(APDS9960_ID)
	if id != APDS9960_ID_1 && id != APDS9960_ID_2 {
		return errors.New("device is not APDS9960")
	}

	// Disable all features
	if err = h.setMode(APDS9960_ALL, APDS9960_OFF); err != nil {
		return
	}

	if err := h.connection.WriteByteData(APDS9960_ATIME, APDS9960_DEFAULT_ATIME); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_WTIME, APDS9960_DEFAULT_WTIME); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_PPULSE, APDS9960_DEFAULT_PROX_PPULSE); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_POFFSET_UR, APDS9960_DEFAULT_POFFSET_UR); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_POFFSET_DL, APDS9960_DEFAULT_POFFSET_DL); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_CONFIG1, APDS9960_DEFAULT_CONFIG1); err != nil {
		return err
	}

	if err := h.setLEDDrive(APDS9960_DEFAULT_LDRIVE); err != nil {
		return err
	}

	if err := h.setProximityGain(APDS9960_DEFAULT_PGAIN); err != nil {
		return err
	}

	if err := h.setAmbientLightGain(APDS9960_DEFAULT_AGAIN); err != nil {
		return err
	}

	if err := h.setProxIntLowThresh(APDS9960_DEFAULT_PILT); err != nil {
		return err
	}

	if err := h.setProxIntHighThresh(APDS9960_DEFAULT_PIHT); err != nil {
		return err
	}

	if err := h.setLightIntLowThreshold(APDS9960_DEFAULT_AILT); err != nil {
		return err
	}

	if err := h.setLightIntHighThreshold(APDS9960_DEFAULT_AIHT); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_PERS, APDS9960_DEFAULT_PERS); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_CONFIG2, APDS9960_DEFAULT_CONFIG2); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_CONFIG3, APDS9960_DEFAULT_CONFIG3); err != nil {
		return err
	}

	if err := h.setGestureEnterThresh(APDS9960_DEFAULT_GPENTH); err != nil {
		return err
	}

	if err := h.setGestureExitThresh(APDS9960_DEFAULT_GEXTH); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_CONFIG1, APDS9960_DEFAULT_CONFIG1); err != nil {
		return err
	}

	if err := h.setGestureGain(APDS9960_DEFAULT_GGAIN); err != nil {
		return err
	}

	if err := h.setGestureLEDDrive(APDS9960_DEFAULT_GLDRIVE); err != nil {
		return err
	}

	if err := h.setGestureWaitTime(APDS9960_DEFAULT_GWTIME); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GOFFSET_U, APDS9960_DEFAULT_GOFFSET); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GOFFSET_D, APDS9960_DEFAULT_GOFFSET); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GOFFSET_L, APDS9960_DEFAULT_GOFFSET); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GOFFSET_R, APDS9960_DEFAULT_GOFFSET); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GPULSE, APDS9960_DEFAULT_GPULSE); err != nil {
		return err
	}

	if err := h.connection.WriteByteData(APDS9960_GCONF3, APDS9960_DEFAULT_GCONF3); err != nil {
		return err
	}

	if err := h.setGestureIntEnable(APDS9960_DEFAULT_GIEN); err != nil {
		return err
	}

	return
}

// Stop apds9960
func (h *APDS9960Driver) Stop() (err error) {
	return
}

// Halt returns true if devices is halted successfully
func (h *APDS9960Driver) Halt() (err error) {
	h.Stop()
	return
}

func (h *APDS9960Driver) getMode() (mode uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	mode, err = h.connection.ReadByteData(APDS9960_ENABLE)
	return
}

func (h *APDS9960Driver) setMode(mode, enable uint8) error {

	regVal, err := h.getMode()
	if err != nil {
		return err
	}
	if regVal == APDS9960_ERROR {
		return errors.New("device not enabled")
	}

	enable = enable & 0x01
	if mode <= 6 {
		if enable > 0 {
			regVal |= 1 << mode
		} else {
			regVal &^= 1 << mode
		}
	} else if mode == APDS9960_ALL {
		if enable > 0 {
			regVal = 0x7F
		} else {
			regVal = 0x00
		}
	}

	if h.connection == nil {
		return errors.New("connection not available")
	}
	return h.connection.WriteByteData(APDS9960_ENABLE, regVal)
}

func (h *APDS9960Driver) getLEDDrive() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	val = (val >> 6) & 0x03 //00000011
	return

}

func (h *APDS9960Driver) setLEDDrive(drive uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	if err != nil {
		return errors.New("not able to read control register")
	}

	drive &= 0x03 //00000011
	drive = drive << 6
	val &= 0x3F //00111111
	val |= drive

	return h.connection.WriteByteData(APDS9960_CONTROL, val)
}

func (h *APDS9960Driver) getGestureIntEnable() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF4)
	val = (val >> 1) & 0x01 //00000001
	return

}

func (h *APDS9960Driver) setGestureIntEnable(enable uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF4)
	if err != nil {
		return errors.New("not able to read control register")
	}

	enable &= 0x01 //00000001
	enable = enable << 1
	val &= 0xFD //11111101
	val |= enable

	return h.connection.WriteByteData(APDS9960_GCONF4, val)
}

func (h *APDS9960Driver) getProximityGain() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	val = (val >> 2) & 0x03 //00000011
	return

}

func (h *APDS9960Driver) setProximityGain(gain uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	if err != nil {
		return errors.New("not able to read control register")
	}

	gain &= 0x03 //00000011
	gain = gain << 2
	val &= 0xF3 //11110011
	val |= gain

	return h.connection.WriteByteData(APDS9960_CONTROL, val)
}

func (h *APDS9960Driver) getAmbientLightGain() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	val &= 0x03 //00000011
	return

}

func (h *APDS9960Driver) setAmbientLightGain(gain uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_CONTROL)
	if err != nil {
		return errors.New("not able to read control register")
	}

	gain &= 0x03 //00000011
	val &= 0xFC  //11111100
	val |= gain

	return h.connection.WriteByteData(APDS9960_CONTROL, val)
}

func (h *APDS9960Driver) getProxIntLowThresh() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_PILT)
	return

}

func (h *APDS9960Driver) setProxIntLowThresh(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_PILT, threshold)
}

func (h *APDS9960Driver) getProxIntHighThresh() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_PIHT)
	return

}

func (h *APDS9960Driver) setProxIntHighThresh(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_PIHT, threshold)
}

func (h *APDS9960Driver) getLightIntLowThreshold() (threshold uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_AILTL)
	if err != nil {
		return
	}

	threshold = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_AILTH)
	if err != nil {
		return
	}

	threshold = threshold + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) setLightIntLowThreshold(threshold uint16) (err error) {

	buf := make([]uint8, 2)
	binary.BigEndian.PutUint16(buf, threshold)

	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.connection.WriteByteData(APDS9960_AILTL, buf[0])
	if err != nil {
		return
	}

	return h.connection.WriteByteData(APDS9960_AILTH, buf[1])
}

func (h *APDS9960Driver) getLightIntHighThreshold() (threshold uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_AIHTL)
	if err != nil {
		return
	}

	threshold = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_AIHTH)
	if err != nil {
		return
	}

	threshold = threshold + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) setLightIntHighThreshold(threshold uint16) (err error) {

	buf := make([]uint8, 2)
	binary.BigEndian.PutUint16(buf, threshold)

	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.connection.WriteByteData(APDS9960_AIHTL, buf[0])
	if err != nil {
		return
	}

	return h.connection.WriteByteData(APDS9960_AIHTH, buf[1])
}

func (h *APDS9960Driver) getGestureEnterThresh() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GPENTH)
	return

}

func (h *APDS9960Driver) setGestureEnterThresh(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_GPENTH, threshold)
}

func (h *APDS9960Driver) getGestureExitThresh() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GEXTH)
	return

}

func (h *APDS9960Driver) setGestureExitThresh(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_GEXTH, threshold)
}

func (h *APDS9960Driver) getGestureGain() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	val = (val >> 5) & 0x03 //00000011
	return

}

func (h *APDS9960Driver) setGestureGain(gain uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	if err != nil {
		return errors.New("not able to read control register")
	}

	gain &= 0x03 //00000011
	gain = gain << 5
	val &= 0x9F //10011111
	val |= gain

	return h.connection.WriteByteData(APDS9960_GCONF2, val)
}

func (h *APDS9960Driver) getGestureLEDDrive() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	val = (val >> 3) & 0x03 //00000011
	return

}

func (h *APDS9960Driver) setGestureLEDDrive(drive uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	if err != nil {
		return errors.New("not able to read control register")
	}

	drive &= 0x03 //00000011
	drive = drive << 3
	val &= 0xE7 //11100111
	val |= drive

	return h.connection.WriteByteData(APDS9960_GCONF2, val)
}

func (h *APDS9960Driver) getGestureWaitTime() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	val &= 0x07 //00000111
	return

}

func (h *APDS9960Driver) setGestureWaitTime(waitTime uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	if err != nil {
		return errors.New("not able to read control register")
	}

	waitTime &= 0x07 //00000111
	val &= 0xF8      //11111000
	val |= waitTime

	return h.connection.WriteByteData(APDS9960_GCONF2, val)
}

func (h *APDS9960Driver) EnableLightSensor(interrupts bool) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.setAmbientLightGain(APDS9960_DEFAULT_AGAIN)
	if err != nil {
		return
	}

	var interruptsInt uint8
	if interrupts {
		interruptsInt = 1
	}
	err = h.setAmbientLightIntEnable(interruptsInt)
	if err != nil {
		return
	}

	err = h.EnablePower()
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_AMBIENT_LIGHT, 1)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) disableLightSensor() (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.setAmbientLightIntEnable(0)
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_AMBIENT_LIGHT, 0)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) EnableProximitySensor(interrupts bool) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.setProximityGain(APDS9960_DEFAULT_PGAIN)
	if err != nil {
		return
	}

	err = h.setLEDDrive(APDS9960_DEFAULT_LDRIVE)
	if err != nil {
		return
	}

	var interruptsInt uint8
	if interrupts {
		interruptsInt = 1
	}
	err = h.setProximityIntEnable(interruptsInt)
	if err != nil {
		return
	}

	err = h.EnablePower()
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_PROXIMITY, 1)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) disableProximitySensor() (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.setProximityIntEnable(0)
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_PROXIMITY, 0)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) EnableGestureSensor(interrupts bool) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	h.resetGestureParameters()
	err = h.connection.WriteByteData(APDS9960_WTIME, 0xFF)
	if err != nil {
		return err
	}

	err = h.connection.WriteByteData(APDS9960_PPULSE, APDS9960_DEFAULT_GESTURE_PPULSE)
	if err != nil {
		return err
	}

	err = h.setLEDBoost(APDS9960_LED_BOOST_300)
	if err != nil {
		return
	}

	var interruptsInt uint8
	if interrupts {
		interruptsInt = 1
	}
	err = h.setGestureIntEnable(interruptsInt)
	if err != nil {
		return
	}

	err = h.setGestureMode(1)
	if err != nil {
		return
	}

	err = h.EnablePower()
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_WAIT, 1)
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_PROXIMITY, 1)
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_GESTURE, 1)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) disableGestureSensor() (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	err = h.setGestureIntEnable(0)
	if err != nil {
		return
	}

	err = h.setGestureMode(0)
	if err != nil {
		return
	}

	err = h.setMode(APDS9960_GESTURE, 0)
	if err != nil {
		return
	}

	return
}

func (h *APDS9960Driver) IsGestureAvailable() (available bool) {
	if h.connection == nil {
		return
	}

	val, err := h.connection.ReadByteData(APDS9960_GSTATUS)
	if err != nil {
		return
	}
	val &= APDS9960_GVALID

	available = val == 1
	return
}

func (h *APDS9960Driver) ReadGesture() int {

	var fifoLevel uint8
	var bytesRead int
	var fifoData []uint8

	available := h.IsGestureAvailable()
	if !available {
		return APDS9960_DIR_NONE
	}

	mode, err := h.getMode()
	if (mode&0x41) == 0 || err != nil {
		return h.gestureAux.motion
	}

	for {
		time.Sleep(APDS9960_FIFO_PAUSE_TIME * time.Millisecond)

		gstatus, err := h.connection.ReadByteData(APDS9960_GSTATUS)
		if err != nil {
			return APDS9960_ERROR
		}

		if (gstatus & APDS9960_GVALID) == APDS9960_GVALID {

			fifoLevel, err = h.connection.ReadByteData(APDS9960_GFLVL)
			if err != nil {
				return APDS9960_ERROR
			}

			fmt.Println(fifoLevel, 4*fifoLevel, len(fifoData), fifoData)

			if fifoLevel > 0 && fifoLevel <= 32 {
				/*err = h.connection.WriteByte(APDS9960_GFIFO_U)
				if err != nil {
					return APDS9960_ERROR
				} */
				//fifoData = make([]uint8, 4*fifoLevel)
				fifoData, bytesRead, err = h.wireReadBlock(APDS9960_GFIFO_U, 4*int(fifoLevel))
				fmt.Println("FIFO DATA", fifoData, len(fifoData))
				if err != nil {
					return APDS9960_ERROR
				}
				if bytesRead > 128 || bytesRead != 4*int(fifoLevel) {
					fmt.Println(bytesRead, fifoLevel, ">>>>>")
					return APDS9960_ERROR
				}

				if bytesRead >= 4 {

					for i := 0; i < bytesRead; i += 4 {
						h.gestureData.u[h.gestureData.index] = fifoData[i+0]
						h.gestureData.d[h.gestureData.index] = fifoData[i+1]
						h.gestureData.l[h.gestureData.index] = fifoData[i+2]
						h.gestureData.r[h.gestureData.index] = fifoData[i+3]
						h.gestureData.index++
						h.gestureData.totalGestures++
						if h.gestureData.index >= 32 {
							break
						}
					}

					h.processGestureData()
					h.decodeGesture()

					h.gestureData.index = 0
					h.gestureData.totalGestures = 0

				}
			}
		} else {
			time.Sleep(APDS9960_FIFO_PAUSE_TIME * time.Millisecond)
			h.decodeGesture()
			h.resetGestureParameters()
			return h.gestureAux.motion
		}
	}
	return h.gestureAux.motion
}

func (h *APDS9960Driver) EnablePower() (err error) {
	err = h.setMode(APDS9960_POWER, 1)
	return
}

func (h *APDS9960Driver) disablePower() (err error) {
	err = h.setMode(APDS9960_POWER, 0)
	return
}

func (h *APDS9960Driver) getGestureMode() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF4)
	val &= 0x01 //00000001
	return

}

func (h *APDS9960Driver) setGestureMode(mode uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF4)
	if err != nil {
		return errors.New("not able to read control register")
	}

	mode &= 0x01 //00000001
	val &= 0xFE  //11111110
	val |= mode

	return h.connection.WriteByteData(APDS9960_GCONF4, val)
}

func (h *APDS9960Driver) clearProximityInt() (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	_, err = h.connection.ReadByteData(APDS9960_PICLEAR)
	if err != nil {
		return errors.New("not able to read control register")
	}

	return
}

func (h *APDS9960Driver) clearAmbientLightInt() (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	_, err = h.connection.ReadByteData(APDS9960_AICLEAR)
	if err != nil {
		return errors.New("not able to read control register")
	}

	return
}

func (h *APDS9960Driver) getProximityIntEnable() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_ENABLE)
	val = (val >> 5) & 0x01 //00000001
	return

}

func (h *APDS9960Driver) setProximityIntEnable(enable uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_ENABLE)
	if err != nil {
		return errors.New("not able to read control register")
	}

	enable &= 0x01 //00000001
	enable = enable << 5
	val &= 0xDF //11011111
	val |= enable

	return h.connection.WriteByteData(APDS9960_ENABLE, val)
}

func (h *APDS9960Driver) getAmbientLightIntEnable() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_ENABLE)
	val = (val >> 4) & 0x01 //00000001
	return

}

func (h *APDS9960Driver) setAmbientLightIntEnable(enable uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_ENABLE)
	if err != nil {
		return errors.New("not able to read control register")
	}

	enable &= 0x01 //00000001
	enable = enable << 4
	val &= 0xEF //11011111
	val |= enable

	return h.connection.WriteByteData(APDS9960_ENABLE, val)
}

func (h *APDS9960Driver) getProximityIntHighThreshold() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.ReadByteData(APDS9960_PIHT)
}

func (h *APDS9960Driver) setProximityIntHighThreshold(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_PIHT, threshold)
}

func (h *APDS9960Driver) getProximityIntLowThreshold() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.ReadByteData(APDS9960_PILT)
}

func (h *APDS9960Driver) setProximityIntLowThreshold(threshold uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	return h.connection.WriteByteData(APDS9960_PILT, threshold)
}

func (h *APDS9960Driver) getProxPhotoMask() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_CONFIG3)
	if err != nil {
		val = APDS9960_ERROR
		return
	}
	val &= 0x0F //00001111
	return

}

func (h *APDS9960Driver) setProxPhotoMask(mask uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_CONFIG3)
	if err != nil {
		return errors.New("not able to read control register")
	}

	mask &= 0x0F //00001111
	val &= 0xF0  //11110000
	val |= mask

	return h.connection.WriteByteData(APDS9960_CONFIG3, val)
}

func (h *APDS9960Driver) resetGestureParameters() {
	h.gestureData.index = 0
	h.gestureData.totalGestures = 0

	var gestureAux apds9960GestureAux
	h.gestureAux = gestureAux
}

func (h *APDS9960Driver) getLEDBoost() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	val = (val >> 4) & 0x03 //00000011
	return

}

func (h *APDS9960Driver) setLEDBoost(boost uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF2)
	if err != nil {
		return errors.New("not able to read control register")
	}

	boost &= 0x03 //00000011
	boost = boost << 4
	val &= 0xCF //11001111
	val |= boost

	return h.connection.WriteByteData(APDS9960_GCONF2, val)
}

func (h *APDS9960Driver) getProxGainCompEnable() (val uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	val, err = h.connection.ReadByteData(APDS9960_GCONF3)
	val = (val >> 5) & 0x01 //00000001
	return

}

func (h *APDS9960Driver) setProxGainCompEnable(enable uint8) (err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GCONF3)
	if err != nil {
		return errors.New("not able to read control register")
	}

	enable &= 0x01 //00000001
	enable = enable << 5
	val &= 0xCF //11001111
	val |= enable

	return h.connection.WriteByteData(APDS9960_GCONF3, val)
}

func (h *APDS9960Driver) ReadAmbientLight() (ambientLight uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_CDATAL)
	if err != nil {
		return
	}

	ambientLight = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_CDATAH)
	if err != nil {
		return
	}

	ambientLight = ambientLight + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) ReadRedLight() (redLight uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_RDATAL)
	if err != nil {
		return
	}

	redLight = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_RDATAH)
	if err != nil {
		return
	}

	redLight = redLight + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) ReadGreenLight() (greenLight uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_GDATAL)
	if err != nil {
		return
	}

	greenLight = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_GDATAH)
	if err != nil {
		return
	}

	greenLight = greenLight + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) ReadBlueLight() (blueLight uint16, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	var val uint8
	val, err = h.connection.ReadByteData(APDS9960_BDATAL)
	if err != nil {
		return
	}

	blueLight = uint16(val)

	val, err = h.connection.ReadByteData(APDS9960_BDATAH)
	if err != nil {
		return
	}

	blueLight = blueLight + (uint16(val) << 8)
	return
}

func (h *APDS9960Driver) ReadProximity() (proximity uint8, err error) {
	if h.connection == nil {
		err = errors.New("connection not available")
		return
	}

	proximity, err = h.connection.ReadByteData(APDS9960_PDATA)
	return
}

func (h *APDS9960Driver) decodeGesture() {

	if h.gestureAux.state == APDS9960_NEAR_STATE {
		h.gestureAux.motion = APDS9960_DIR_NEAR
		return
	} else if h.gestureAux.state == APDS9960_FAR_STATE {
		h.gestureAux.motion = APDS9960_DIR_FAR
		return
	}

	if h.gestureAux.udCount == -1 && h.gestureAux.lrCount == 0 {
		h.gestureAux.motion = APDS9960_DIR_UP
	} else if h.gestureAux.udCount == 1 && h.gestureAux.lrCount == 0 {
		h.gestureAux.motion = APDS9960_DIR_DOWN
	} else if h.gestureAux.udCount == 0 && h.gestureAux.lrCount == 1 {
		h.gestureAux.motion = APDS9960_DIR_RIGHT
	} else if h.gestureAux.udCount == 0 && h.gestureAux.lrCount == -1 {
		h.gestureAux.motion = APDS9960_DIR_LEFT
	} else if h.gestureAux.udCount == -1 && h.gestureAux.lrCount == 1 {
		if math.Abs(float64(h.gestureAux.udCount)) > math.Abs(float64(h.gestureAux.lrCount)) {
			h.gestureAux.motion = APDS9960_DIR_UP
		} else {
			h.gestureAux.motion = APDS9960_DIR_RIGHT
		}
	} else if h.gestureAux.udCount == 1 && h.gestureAux.lrCount == -1 {
		if math.Abs(float64(h.gestureAux.udCount)) > math.Abs(float64(h.gestureAux.lrCount)) {
			h.gestureAux.motion = APDS9960_DIR_DOWN
		} else {
			h.gestureAux.motion = APDS9960_DIR_LEFT
		}
	} else if h.gestureAux.udCount == -1 && h.gestureAux.lrCount == -1 {
		if math.Abs(float64(h.gestureAux.udCount)) > math.Abs(float64(h.gestureAux.lrCount)) {
			h.gestureAux.motion = APDS9960_DIR_UP
		} else {
			h.gestureAux.motion = APDS9960_DIR_LEFT
		}
	} else if h.gestureAux.udCount == 1 && h.gestureAux.lrCount == 1 {
		if math.Abs(float64(h.gestureAux.udCount)) > math.Abs(float64(h.gestureAux.lrCount)) {
			h.gestureAux.motion = APDS9960_DIR_DOWN
		} else {
			h.gestureAux.motion = APDS9960_DIR_RIGHT
		}
	}
}

func (h *APDS9960Driver) processGestureData() {

	var uFirst uint8
	var dFirst uint8
	var lFirst uint8
	var rFirst uint8
	var uLast uint8
	var dLast uint8
	var lLast uint8
	var rLast uint8
	var udRatioFirst int
	var lrRatioFirst int
	var udRatioLast int
	var lrRatioLast int
	var udDelta int
	var lrDelta int

	if h.gestureData.totalGestures <= 4 {
		return
	}
	if h.gestureData.totalGestures <= 32 && h.gestureData.totalGestures > 0 {

		var i int
		for i = 0; i < int(h.gestureData.totalGestures); i++ {
			if h.gestureData.u[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.d[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.l[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.r[i] > APDS9960_GESTURE_THRESHOLD_OUT {
				uFirst = h.gestureData.u[i]
				dFirst = h.gestureData.d[i]
				lFirst = h.gestureData.l[i]
				rFirst = h.gestureData.r[i]
				break
			}
		}

		if uFirst == 0 || dFirst == 0 || lFirst == 0 || rFirst == 0 {
			return
		}

		for i = int(h.gestureData.totalGestures) - 1; i >= 0; i-- {
			if h.gestureData.u[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.d[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.l[i] > APDS9960_GESTURE_THRESHOLD_OUT &&
				h.gestureData.r[i] > APDS9960_GESTURE_THRESHOLD_OUT {
				uLast = h.gestureData.u[i]
				dLast = h.gestureData.d[i]
				lLast = h.gestureData.l[i]
				rLast = h.gestureData.r[i]
				break
			}
		}
	}

	udRatioFirst = int(((float32(uFirst) - float32(dFirst)) * 100) / (float32(uFirst) + float32(dFirst)))
	lrRatioFirst = int(((float32(lFirst) - float32(rFirst)) * 100) / (float32(lFirst) + float32(rFirst)))
	udRatioLast = int(((float32(uLast) - float32(dLast)) * 100) / (float32(uLast) + float32(dLast)))
	lrRatioLast = int(((float32(lLast) - float32(rLast)) * 100) / (float32(lLast) + float32(rLast)))

	fmt.Println("UDRF", udRatioFirst, "UF", uFirst, "UL", uLast, "UDRL", udRatioLast, "TG", h.gestureData.totalGestures)

	udDelta = udRatioLast - udRatioFirst
	lrDelta = lrRatioLast - lrRatioFirst

	h.gestureAux.udDelta += udDelta
	h.gestureAux.lrDelta += lrDelta

	if h.gestureAux.udDelta >= APDS9960_GESTURE_SENSITIVITY_1 {
		h.gestureAux.udCount = 1
	} else if h.gestureAux.udDelta <= -APDS9960_GESTURE_SENSITIVITY_1 {
		h.gestureAux.udCount = -1
	} else {
		h.gestureAux.udCount = 0
	}

	if h.gestureAux.lrDelta >= APDS9960_GESTURE_SENSITIVITY_1 {
		h.gestureAux.lrCount = 1
	} else if h.gestureAux.lrDelta <= -APDS9960_GESTURE_SENSITIVITY_1 {
		h.gestureAux.lrCount = -1
	} else {
		h.gestureAux.lrCount = 0
	}

	if h.gestureAux.udCount == 0 && h.gestureAux.lrCount == 0 {
		if int(math.Abs(float64(udDelta))) < APDS9960_GESTURE_SENSITIVITY_2 && int(math.Abs(float64(lrDelta))) < APDS9960_GESTURE_SENSITIVITY_2 {
			if udDelta == 0 && lrDelta == 0 {
				h.gestureAux.nearCount++
			} else if udDelta != 0 || lrDelta != 0 {
				h.gestureAux.farCount++
			}

			if h.gestureAux.nearCount >= 10 && h.gestureAux.farCount >= 2 {
				if udDelta == 0 && lrDelta == 0 {
					h.gestureAux.state = APDS9960_NEAR_STATE
				} else if udDelta != 0 && lrDelta != 0 {
					h.gestureAux.state = APDS9960_FAR_STATE
				}
			}

		}
	} else {
		if int(math.Abs(float64(udDelta))) < APDS9960_GESTURE_SENSITIVITY_2 && int(math.Abs(float64(lrDelta))) < APDS9960_GESTURE_SENSITIVITY_2 {
			if udDelta == 0 && lrDelta == 0 {
				h.gestureAux.nearCount++
				if h.gestureAux.nearCount >= 10 {
					h.gestureAux.udCount = 0
					h.gestureAux.lrCount = 0
					h.gestureAux.udDelta = 0
					h.gestureAux.lrDelta = 0
				}
			}
		}
	}

}

func (h *APDS9960Driver) wireReadBlock(reg uint8, length int) (fifoData []uint8, bytesRead int, err error) {
	blockSize := 32
	fifoData = make([]uint8, length)
	var rb int
	var k int

	err = h.connection.WriteByte(reg)
	if err != nil {
		return
	}

	for length > 0 {
		bufLength := int(math.Min(float64(blockSize), float64(length)))
		buf := make([]uint8, bufLength)

		rb, err = h.connection.Read(buf)
		if err != nil || rb != bufLength {
			return
		}
		for i := 0; i < bufLength; i++ {
			fifoData[k+i] = buf[i]
		}
		k++
		length -= blockSize
		bytesRead += rb
	}
	return
}
