package api

import (
	"fmt"

	"github.com/hybridgroup/gobot"
)

type testDriver struct {
	name       string
	pin        string
	connection gobot.Connection
	gobot.Commander
}

func (t *testDriver) Start() (errs []error)        { return }
func (t *testDriver) Halt() (errs []error)         { return }
func (t *testDriver) Name() string                 { return t.name }
func (t *testDriver) Pin() string                  { return t.pin }
func (t *testDriver) Connection() gobot.Connection { return t.connection }

func newTestDriver(adaptor *testAdaptor, name string, pin string) *testDriver {
	t := &testDriver{
		name:       name,
		connection: adaptor,
		pin:        pin,
		Commander:  gobot.NewCommander(),
	}

	t.AddCommand("TestDriverCommand", func(params map[string]interface{}) interface{} {
		name := params["name"].(string)
		return fmt.Sprintf("hello %v", name)
	})

	t.AddCommand("DriverCommand", func(params map[string]interface{}) interface{} {
		name := params["name"].(string)
		return fmt.Sprintf("hello %v", name)
	})

	return t
}

type testAdaptor struct {
	name string
	port string
}

var testAdaptorConnect = func() (errs []error) { return }
var testAdaptorFinalize = func() (errs []error) { return }

func (t *testAdaptor) Finalize() (errs []error) { return testAdaptorFinalize() }
func (t *testAdaptor) Connect() (errs []error)  { return testAdaptorConnect() }
func (t *testAdaptor) Name() string             { return t.name }
func (t *testAdaptor) Port() string             { return t.port }

func newTestAdaptor(name string, port string) *testAdaptor {
	return &testAdaptor{
		name: name,
		port: port,
	}
}

func newTestRobot(name string) *gobot.Robot {
	adaptor1 := newTestAdaptor("Connection1", "/dev/null")
	adaptor2 := newTestAdaptor("Connection2", "/dev/null")
	adaptor3 := newTestAdaptor("", "/dev/null")
	driver1 := newTestDriver(adaptor1, "Device1", "0")
	driver2 := newTestDriver(adaptor2, "Device2", "2")
	driver3 := newTestDriver(adaptor3, "", "1")
	work := func() {}
	r := gobot.NewRobot(name,
		[]gobot.Connection{adaptor1, adaptor2, adaptor3},
		[]gobot.Device{driver1, driver2, driver3},
		work,
	)
	r.AddCommand("robotTestFunction", func(params map[string]interface{}) interface{} {
		message := params["message"].(string)
		robot := params["robot"].(string)
		return fmt.Sprintf("hey %v, %v", robot, message)
	})
	return r
}
