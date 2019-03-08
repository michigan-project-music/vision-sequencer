package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"os"
	"sort"
	"sync"
	"time"

	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/firmata"
)

// Output pins
const MaxGridLen = 2000
const LedQuadrant1 = "1"
const LedQuadrant2 = "2"
const LedQuadrant3 = "3"
const LedQuadrant4 = "4"
const FrequencyPin = "5"
const FilterPin = "6"
const GatePin = "7"
const RepeatingLed = "8"

// Input pins
const PressurePin = "1"

type Block struct {
	X        int     // From 0 - MaxGridLen
	Y        float64 // From 0 to 1
	Z        float64
	Rotation float64
	ID       int
}
type Grid struct {
	sync.Mutex
	LoopLength int
	Data       []Block
}

type LoopMode int

const (
	Forward LoopMode = iota
	Backward
	PingPong
)

type Sequencer struct {
	sync.Mutex
	CurrentTime       int
	DeltaMS           float64 // Milliseconds per tick
	Grid              *Grid
	FirmataDriver     *firmata.Adaptor
	LoopMode          LoopMode
	Repeating         bool
	RepeatStart       int
	RepeatEnd         int
	RepeatCurrentTime int
	PingPongState     bool // true when increasing
}

func NewGrid() *Grid {
	g := Grid{}
	g.Data = make([]Block, 0)
	return &g
}

func NewSequencer(delta float64, f *firmata.Adaptor) *Sequencer {
	g := NewGrid()
	s := &Sequencer{sync.Mutex{}, 0, delta, g, f, Forward, false, 0, 0, 0, true}
	return s
}

func (g *Grid) Unpack(JsonData string) {
	g.Lock()
	defer g.Unlock()
	json.Unmarshal([]byte(JsonData), g)
	fmt.Println(g.LoopLength)
	// if err != nil {
	// 	fmt.Println(JsonData, err)
	// }
	// fmt.Println(g.ToString())
}

func (b Block) ToString() string {
	return fmt.Sprintf("x: %d, y: %f, z: %f, rot: %f, id: %d", b.X, b.Y, b.Z, b.Rotation, b.ID)
}
func (b Block) Play(f *firmata.Adaptor) {
	// Map x y z rot id to CV values here
	// For now, let's just print it out
	fmt.Printf("Playing %s\n", b.ToString())
	f.PwmWrite(FrequencyPin, byte(int(b.Y*50)+100))
	f.PwmWrite(FilterPin, byte(int(b.Rotation*255))) // TODO: i don't know what range the rotation is
	go func() {
		f.DigitalWrite(GatePin, 1)
		time.Sleep(5 * time.Millisecond)
		f.DigitalWrite(GatePin, 0)
	}()
}

func (g *Grid) ToString() string {
	ret := fmt.Sprintf("loop: %d, len: %d\n", g.LoopLength, len(g.Data))
	for i := range g.Data {
		ret += "\t" + g.Data[i].ToString() + "\n"
	}
	return ret
}

func (s *Sequencer) Tick(ticker *time.Ticker) {
	s.Grid.Lock()
	s.Lock()
	defer s.Grid.Unlock()
	if s.Grid.LoopLength == 0 || len(s.Grid.Data) == 0 {
		s.Unlock()
		return
	}
	switch s.LoopMode {
	case Forward:
		s.CurrentTime = (s.CurrentTime + 1) % s.Grid.LoopLength
	case Backward:
		s.CurrentTime = (s.CurrentTime - 1 + s.Grid.LoopLength) % s.Grid.LoopLength
	case PingPong:
		if s.PingPongState {
			if s.CurrentTime == s.Grid.LoopLength {
				s.PingPongState = !s.PingPongState
			} else {
				s.CurrentTime++
			}
		} else {
			if s.CurrentTime == 0 {
				s.PingPongState = !s.PingPongState
			} else {
				s.CurrentTime--
			}
		}
	}

	if s.CurrentTime == 0 {
		go func() {
			s.FirmataDriver.DigitalWrite(LedQuadrant1, 1)
			time.Sleep(100 * time.Millisecond)
			s.FirmataDriver.DigitalWrite(LedQuadrant1, 0)
		}()
	}

	if s.CurrentTime == MaxGridLen/4 {
		go func() {
			s.FirmataDriver.DigitalWrite(LedQuadrant2, 1)
			time.Sleep(100 * time.Millisecond)
			s.FirmataDriver.DigitalWrite(LedQuadrant2, 0)
		}()
	}

	if s.CurrentTime == 2*MaxGridLen/4 {
		go func() {
			s.FirmataDriver.DigitalWrite(LedQuadrant3, 1)
			time.Sleep(100 * time.Millisecond)
			s.FirmataDriver.DigitalWrite(LedQuadrant3, 0)
		}()
	}

	if s.CurrentTime == 3*MaxGridLen/4 {
		go func() {
			s.FirmataDriver.DigitalWrite(LedQuadrant4, 1)
			time.Sleep(100 * time.Millisecond)
			s.FirmataDriver.DigitalWrite(LedQuadrant4, 0)
		}()
	}

	if s.Repeating {
		// If repeating, increment RepeatTime and play notes from that instead
		s.RepeatCurrentTime++

		if s.RepeatCurrentTime == s.RepeatEnd {
			s.RepeatCurrentTime = s.RepeatStart
			fmt.Println("Repeat range: ", ((s.RepeatEnd+s.Grid.LoopLength)-s.RepeatStart)%s.Grid.LoopLength)
		}
		i := sort.Search(len(s.Grid.Data), func(i int) bool { return s.Grid.Data[i].X >= s.RepeatCurrentTime })
		if i >= 0 && i < len(s.Grid.Data) && s.Grid.Data[i].X == s.RepeatCurrentTime {
			// This block is happening NOW
			s.Grid.Data[i].Play(s.FirmataDriver)
		} else {
			// it's in the future, so we don't care
		}
		s.Unlock()
		return
	}
	s.Unlock()

	if s.CurrentTime == 0 {
		fmt.Println(s.Grid.ToString())
	}
	// fmt.Println(s.CurrentTime)
	// Find index of block to play
	i := sort.Search(len(s.Grid.Data), func(i int) bool { return s.Grid.Data[i].X >= s.CurrentTime })
	// i is the smallest index of the block. Now we just have to make sure it's the current one
	// fmt.Println(i)
	// fmt.Println(s.Grid.Data)
	if i >= 0 && i < len(s.Grid.Data) && s.Grid.Data[i].X == s.CurrentTime {
		// This block is happening NOW
		s.Grid.Data[i].Play(s.FirmataDriver)
	} else {
		// it's in the future, so we don't care
	}
}

func (s *Sequencer) HandleArduinoInputs() {
	// Check all inputs to Arduino, and make changes to loop length, etc.
	// s.Lock()
	// defer s.Unlock()
	// val, err := s.FirmataDriver.AnalogRead(PressurePin)
	// if err == nil {
	// 	if val > 10 {
	// 		s.FirmataDriver.DigitalWrite(RepeatingLed, 0)
	// 		s.Repeating = false
	// 	} else {
	// 		if val > 300 {
	// 			val = 300
	// 		}
	// 		val = 300 - val + 50
	// 		if s.Repeating {
	// 			// Already been repeating
	// 			s.RepeatStart = (s.RepeatEnd - int((float64(val) / 200.0 * float64(s.Grid.LoopLength))) + s.Grid.LoopLength) % s.Grid.LoopLength
	// 		} else {
	// 			// Just started repeating
	// 			s.FirmataDriver.DigitalWrite(RepeatingLed, 1)
	// 			s.Repeating = true
	// 			s.RepeatStart = (s.CurrentTime - int((float64(val) / 200.0 * float64(s.Grid.LoopLength))) + s.Grid.LoopLength) % s.Grid.LoopLength
	// 			s.RepeatEnd = s.CurrentTime
	// 			s.RepeatCurrentTime = s.RepeatStart
	// 		}
	// 	}
	// } else {
	// 	fmt.Println(err)
	// }
}

func main() {
	delta := 1. // 1 ms per cycle (MaxGridLen cycles for the entire length of the board)

	firmataAdaptor := firmata.NewAdaptor(os.Args[1])
	s := NewSequencer(delta, firmataAdaptor)
	// s.FirmataDriver.On

	// firmataI2c := firmata.NewFirmataI2cConnection(firmataAdaptor, 1)
	// firmataI2c.
	work := func() {
		gobot.Every(10*time.Millisecond, func() {
			s.HandleArduinoInputs()
		})
	}

	robot := gobot.NewRobot("bot",
		[]gobot.Connection{firmataAdaptor},
		[]gobot.Device{},
		work,
	)

	go robot.Start()
	time.Sleep(5 * time.Second) // lol

	ticker := time.NewTicker(time.Duration(delta) * time.Millisecond)
	exit := make(chan interface{})

	go func() { // Read input from python pipe
		for {
			reader := bufio.NewReader(os.Stdin)
			data, _ := reader.ReadString('\n')
			s.Grid.Unpack(data)
		}
	}()

	for { // Main time loop for sequencer
		select {
		case <-ticker.C:
			go s.Tick(ticker)
		case <-exit:
			fmt.Println("Shutting down...")
			ticker.Stop()
			return
		}
	}
}
