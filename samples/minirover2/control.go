package main

import (
	"fmt"
	"io"
	"log"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/jacobsa/go-serial/serial"

	"github.com/echolabsinc/robotcore/rcutil"
)

func findPort() (string, error) {
	for _, possibleFile := range []string{"/dev/ttyTHS0"} {
		_, err := os.Stat(possibleFile)
		if err == nil {
			return possibleFile, nil
		}
	}

	lines, err := rcutil.ExecuteShellCommand("arduino-cli", "board", "list")
	if err != nil {
		return "", err
	}

	for _, l := range lines {
		if strings.Index(l, "Mega") < 0 {
			continue
		}
		return strings.Split(l, " ")[0], nil
	}

	return "", fmt.Errorf("couldn't find an arduino")
}

func getSerialConfig() (serial.OpenOptions, error) {

	options := serial.OpenOptions{
		PortName:        "",
		BaudRate:        9600,
		DataBits:        8,
		StopBits:        1,
		MinimumReadSize: 4,
	}

	portName, err := findPort()
	if err != nil {
		return options, err
	}

	options.PortName = portName

	return options, nil
}

// ------

type Rover struct {
	out io.Writer
}

func (r *Rover) Forward(power int) error {
	s := fmt.Sprintf("0f%d\r"+
		"1f%d\r"+
		"2f%d\r"+
		"3f%d\r", power, power, power, power)
	_, err := r.out.Write([]byte(s))
	return err
}

func (r *Rover) Backward(power int) error {
	s := fmt.Sprintf("0b%d\r"+
		"1b%d\r"+
		"2b%d\r"+
		"3b%d\r", power, power, power, power)
	_, err := r.out.Write([]byte(s))
	return err
}

func (r *Rover) Stop() error {
	s := fmt.Sprintf("0s\r" +
		"1s\r" +
		"2s\r" +
		"3s\r")
	_, err := r.out.Write([]byte(s))
	return err
}

func (r *Rover) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	var err error

	speed := 64
	if req.FormValue("speed") != "" {
		// TODO
	}

	switch req.FormValue("a") {
	case "f":
		err = r.Forward(speed)
	case "b":
		err = r.Backward(speed)
	case "s":
		err = r.Stop()
	default:
		io.WriteString(w, "unknown command: "+req.FormValue("a"))
		return
	}

	if err != nil {
		io.WriteString(w, "err: "+err.Error())
		return
	}

	io.WriteString(w, "done")
}

// ------

func main() {
	options, err := getSerialConfig()
	if err != nil {
		log.Fatalf("can't get serial config: %v", err)
	}

	port, err := serial.Open(options)
	if err != nil {
		log.Fatalf("can't option serial port %v", err)
	}
	defer port.Close()

	time.Sleep(1000 * time.Millisecond) // wait for startup?

	fmt.Printf("ready\n")

	rover := Rover{port}
	defer rover.Stop()

	if false {
		speed := 50
		for {
			err = rover.Forward(speed)
			if err != nil {
				log.Fatalf("couldn't move rover %v", err)
			}

			time.Sleep(2000 * time.Millisecond)

			err = rover.Stop()
			if err != nil {
				log.Fatalf("couldn't stop rover %v", err)
			}

			time.Sleep(2000 * time.Millisecond)

			err = rover.Backward(speed)
			if err != nil {
				log.Fatalf("couldn't move rover %v", err)
			}

			time.Sleep(2000 * time.Millisecond)
		}
		return
	}

	if true {
		mux := http.NewServeMux()
		mux.Handle("/api/rover", &rover)
		log.Println("going to listen")
		log.Fatal(http.ListenAndServe(":8080", mux))
	}

}
