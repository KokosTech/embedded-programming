# Embedded Programming

_Любителски Предмет_



## External Interrupts (pin 2 or 3)

- EI**MSK** - could be set INT0 or INT1, used to enable or disable interrupt
- EI**FR** - could be set INTF0, INTF1, used for interrupt state
- EI**CRA** - could be set ISC00/01/10/11, how interrupt should trigger

```c
cli();	// disable interrupts

EIMSK |= (1 << INT0);	// enable external interrupt
EIFR  |= (1 << INTF1);	// clear flag register - if 0 - runs handler
EICRA |= (1 << ISC11);	// when the interrupt should be triggered
EICRA &= ~(1 << ISC10); // continuation of EICRA

sei();	// enable interrupts

...

ISR(INT0_vect) {}	// interrupt handler
```

## Pin Change Interrupts

- PCI**CR** - could be set PCIE2, PCIE1, PCIE0, used for enabling interrupt
- PCI**FR** - could be set PCIF2, PCIF1, PCIF0, used for interrupt state
- PS**MSK**<u>x</u> - x could be 0, 1 or 2, could be set PCINT0-23, used for enabling interrupt for GPIO

```c
cli();

PCICR  |= (1 << PCIE0);
PCIFR  |= (1 << PCIF0);
PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2) | (1 << PCINT3);
// PCMSK0 could be simplifed to PCMSK0 |= 0b00001111 or 0x0f
    
sei();

ISR(PCINT0_vect) {} // interrupt handler
```

## GPIO with registers

- PORTxn - if input - 1 = pull-up, 0 = no pull-up; if output - 1 = high, 0 = low
- DDRx - if 0 = input; if 1 = output
- PINx - used for reading input pin

```c
// write example
DDRB |= (1 << DDB3); // set pin to output
PORTB |= (1 << PORTB3); // output HIGH
```



```c
// read example
DDRB &= ~(1 << DDB3); // set pin to input
bool output = PINB & (1 << PB3) // read output of
```

