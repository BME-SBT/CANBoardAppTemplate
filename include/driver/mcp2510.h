#ifndef MCP2510_H
#define MCP2510_H

#include "platform.h"
#include "log.h"

// Config mode registers
#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2a
#define REG_TXRTSCTRL 0x0d // TXxRTS pin control and status register
#define REG_BFPCTRL 0x0c   // RXnBF pin control and status register

#define REG_CANCTRL (0x0f)       // CAN control register
#define REG_CANSTAT (0x0e)       // CAN control register
#define CANCTRL_REQOP_3 (1 << 7) // 000 = normal, 001 = sleep, 010 = loopback, 011 = listen-only, 100 = config, 111 = POWERUP
#define CANCTRL_ABAT (1 << 4)    // Abort all transmissions
#define CANSTAT_OPMOD_3 (1 << 7) // Operating mode, same as CANCTRL_REQOP_3
#define CANSTAT_ICOD_3 (1 << 3)  // Interrupt flag code: 000 = no interrupt, 001 = error interrupt, 010 = wakeup, 011 = txb0, 100 = txb1, 101 = txb2, 110 = rxb0, 111 = rxb1

// Transmit control buffers
#define REG_TXBnCTRL(n) (0x30 + (n * 0x10)) // Transmit control register
#define REG_TXBnSIDH(n) (0x31 + (n * 0x10)) // Transmit standard id high, from ID<10:3>
#define REG_TXBnSIDL(n) (0x32 + (n * 0x10)) // Transmit standard id low, from ID<2:0> = bit<7:5>, EID<17:16> = bit<1:0>
#define REG_TXBnEID8(n) (0x33 + (n * 0x10)) // Transmit extended id mid, from EID<15:8>
#define REG_TXBnEID0(n) (0x34 + (n * 0x10)) // Transmit extended id low, from EID<7:0>
#define REG_TXBnDLC(n) (0x35 + (n * 0x10))  // Transmit control register

// Transmit data buffer
#define REG_TXBnD0(n) (0x36 + (n * 0x10)) // Transmit buffer databyte 0

// Transmit control flags
#define TXBNCTRL_ABTF (1 << 6)   // 1 = Message aborted, R
#define TXBNCTRL_MLOA (1 << 5)   // 1 = Message lost arbitration, R
#define TXBNCTRL_TXERR (1 << 4)  // 1 = Transmission error, R
#define TXBNCTRL_TXREQ (1 << 3)  // 1 = Request transmission, R/W
#define TXBNCTRL_PRIO_2 (1 << 1) // 00,01,10,11 = priority (11 highest), 2 bits, R/W
#define TXBNSIDL_EXIDE (1 << 3)  // Extended id enable
#define TXBNDCL_RTR (1 << 6)     // Remote transmit request
#define TXBNDCL_DLC_4 (1 << 3)   // Data length code bits,

// Receive buffers
#define REG_RXBnCTRL(n) (0x60 + (n * 0x10)) // Receive control register
#define REG_RXBnSIDH(n) (0x61 + (n * 0x10)) // Receive standard id high, from ID<10:3>
#define REG_RXBnSIDL(n) (0x62 + (n * 0x10)) // Receive standard id low, from ID<2:0> = bit<7:5>, EID<17:16> = bit<1:0>
#define REG_RXBnEID8(n) (0x63 + (n * 0x10)) // Receive extended id mid, from EID<15:8>
#define REG_RXBnEID0(n) (0x64 + (n * 0x10)) // Receive extended id low, from EID<7:0>
#define REG_RXBnDLC(n) (0x65 + (n * 0x10))  // Receive control register

// Receive data buffer
#define REG_RXBnD0(n) (0x66 + (n * 0x10)) // Transmit buffer databyte 0

// Receive control flags
#define RXBNCTRL_RXM_2 (1 << 6)    // Receive buffer operating mode, 11=mask off, 10=only valid eid, 01=only valid id, 00=all valid
#define RXBNCTRL_RXRTR (1 << 3)    // 1 = RTR received
#define RXB0CTRL_BUKT (1 << 2)     // 1 = roll message over to RX1 if valid but RX0 is full
#define RXBNCTRL_FILHIT0 (1 << 0)  // Which acceptance filter got hit - RX0
#define RXB1CTRL_FILHIT_3 (1 << 2) // Which acceptance filter got hit - RX1

#define RXBNSIDL_SRR (1 << 4)  // Standard frame RTR bit
#define RXBNSIDL_IDE (1 << 3)  // 1 = Received extended frame
#define RXBNDLC_RTR (1 << 6)   // 1 = Received extended RTR
#define RXBNDCL_DLC_4 (1 << 3) // Data length code bits

// Filters
#define REG_RXFnSIDH(n) (0x00 + (n * 4))
#define REG_RXFnSIDL(n) (0x01 + (n * 4))
#define REG_RXFnEID8(n) (0x02 + (n * 4))
#define REG_RXFnEIF0(n) (0x03 + (n * 4))

#define RXFNSIDL_EXIDE (1 << 3) // 1 = Extended ID enable

// Masks
#define REG_RXMnSIDH (0x20 + (n * 4))
#define REG_RXMnSIDL (0x21 + (n * 4))
#define REG_RXMnEID8 (0x22 + (n * 4))
#define REG_RXMnEID0 (0x23 + (n * 4))

// Interrupt stuff
#define REG_CANINTE (0x2b)
#define REG_CANINTF (0x2c)

#define CANINTF_RXnIE(n) (0x01 << n)
#define CANINTF_RXnIF(n) (0x01 << n)
#define CANINTF_TXnIF(n) (0x04 << n)

class MCP2510
{

public:
    MCP2510(int cs, int interrupt, long clock_freq, long spi_freq) : m_cs_pin(cs),
                                                                     m_int_pin(interrupt),
                                                                     m_clock_freq(clock_freq)
    {
        m_settings = SPISettings(spi_freq, BitOrder::MSBFIRST, SPI_MODE0);
    }

    int begin(long can_baud)
    {
        // initialize chipselect
        pinMode(m_cs_pin, OUTPUT);

        // Reset the controller
        reset();

        // enter configuration mode
        write_register(REG_CANCTRL, 0x80);
        if ((read_register(REG_CANSTAT) & 0b11100000) != 0b10000000)
        {
            return -1;
        }

        if (setup_clock_magic(can_baud) < 0)
        {
            return -1;
        }

        write_register(REG_CANINTE, 0x00);     // no interrupts
        write_register(REG_BFPCTRL, 0x00);     // disable RX pins
        write_register(REG_TXRTSCTRL, 0x00);   // disable TX pins
        write_register(REG_RXBnCTRL(0), 0x60); // receive all frames
        write_register(REG_RXBnCTRL(1), 0x60); // receive all frames

        // enable normal mode
        write_register(REG_CANCTRL, 0b00000000);
        if ((read_register(REG_CANSTAT) & 0b11100000) != 0b00000000)
        {
            return -1;
        }

        return 0;
    }

    int begin_packet(int id, int dlc, bool rtr)
    {
        if (id < 0 || id > 0x7ff)
        {
            return -1;
        }
        if (dlc > 8 || dlc < 0)
        {
            return -1;
        }

        m_tx_packet_begun = true;
        m_tx_packet_buffer.reset();
        m_tx_packet_buffer.sid = id & 0x7ff;
        m_tx_packet_buffer.rtr = rtr;
        m_tx_packet_buffer.dlc = dlc;

        return 0;
    }

    int write(uint8_t byte)
    {
        return write(&byte, 1);
    }

    int write(uint8_t *buffer, size_t len)
    {
        if (!m_tx_packet_begun)
        {
            return -1;
        }
        if (len + m_tx_packet_buffer.length > 8)
        {
            len = 8 - m_tx_packet_buffer.length;
        }

        memcpy(&m_tx_packet_buffer.data[m_tx_packet_buffer.length], buffer, len);
        m_tx_packet_buffer.length += len;

        return len;
    }

    int end_packet()
    {
        if (!m_tx_packet_begun)
        {
            log("no packet start");
            return -1;
        }
        m_tx_packet_begun = false;

        if (m_tx_packet_buffer.dlc > 0)
        {
            m_tx_packet_buffer.length = m_tx_packet_buffer.dlc;
        }

        // Try buffers
        for (int i = 0; i < 3; i++)
        {
            int buffer_status = read_register(REG_TXBnCTRL(i));
            if ((buffer_status & TXBNCTRL_TXREQ) != 0)
            {
                // buffer in use :(
                continue;
            }

            if (!m_tx_packet_buffer.extended)
            {
                SerialUSB.print("ID: ");
                SerialUSB.print(m_tx_packet_buffer.sid >> 3 & 0xff, BIN);
                SerialUSB.print(m_tx_packet_buffer.sid << 5 & 0xff, BIN);
                SerialUSB.println();
                write_register(REG_TXBnSIDH(i), m_tx_packet_buffer.sid >> 3);
                write_register(REG_TXBnSIDL(i), m_tx_packet_buffer.sid << 5);
                write_register(REG_TXBnEID8(i), 0x00);
                write_register(REG_TXBnEID0(i), 0x00);
            }
            else
            {
                write_register(REG_TXBnSIDH(i), m_tx_packet_buffer.eid >> 21);
                write_register(REG_TXBnSIDL(i), (((m_tx_packet_buffer.eid >> 18) & 0x07) << 5) | TXBNSIDL_EXIDE | ((m_tx_packet_buffer.eid >> 16) & 0x03));
                write_register(REG_TXBnEID8(i), (m_tx_packet_buffer.eid >> 8) & 0xff);
                write_register(REG_TXBnEID0(i), m_tx_packet_buffer.eid & 0xff);
            }

            if (m_tx_packet_buffer.rtr)
            {
                write_register(REG_TXBnDLC(i), TXBNDCL_RTR | m_tx_packet_buffer.length);
            }
            else
            {
                write_register(REG_TXBnDLC(i), m_tx_packet_buffer.length);
            }
            // write data
            for (int b = 0; b < m_tx_packet_buffer.length; b++)
            {
                write_register(REG_TXBnD0(i) + b, m_tx_packet_buffer.data[b]);
            }

            // send it
            write_register(REG_TXBnCTRL(i), TXBNCTRL_TXREQ);

            bool aborted = false;
            while (read_register(REG_TXBnCTRL(i)) & TXBNCTRL_TXREQ)
            {
                if (read_register(REG_TXBnCTRL(i)) & TXBNCTRL_TXERR)
                {
                    aborted = true;
                    // abort all
                    modify_register(REG_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT);
                    break;
                }
                yield();
            }

            if (aborted)
            {
                // clear abort
                modify_register(REG_CANCTRL, CANCTRL_ABAT, 0);
            }
            // clear interrupt
            modify_register(REG_CANINTF, CANINTF_TXnIF(i), 0);
            int result = read_register(REG_TXBnCTRL(i));
            return result & 0b01110000 ? -3 : 0;
        }
        return -2;
    }

    void reset()
    {
        SPIWriter writer(m_settings, m_cs_pin);
        writer.transfer(0xc0);
        delayMicroseconds(10);
    }

private:
    struct PacketData
    {
        int sid;
        bool extended = false;
        int eid;
        bool rtr;
        int dlc;
        int length;
        uint8_t data[8];

        void reset()
        {
            sid = 0;
            extended = false;
            eid = 0;
            rtr = false;
            dlc = 0;
            length = 0;
            memset(data, 0, 8);
        }
    };

    int
    setup_clock_magic(long baudRate)
    {
        static const struct
        {
            long clockFrequency;
            long baudRate;
            uint8_t cnf[3];
        } CNF_MAPPER[] = {
            {(long)8E6, (long)1000E3, {0x00, 0x80, 0x00}},
            {(long)8E6, (long)500E3, {0x00, 0x90, 0x02}},
            {(long)8E6, (long)250E3, {0x00, 0xb1, 0x05}},
            {(long)8E6, (long)200E3, {0x00, 0xb4, 0x06}},
            {(long)8E6, (long)125E3, {0x01, 0xb1, 0x05}},
            {(long)8E6, (long)100E3, {0x01, 0xb4, 0x06}},
            {(long)8E6, (long)80E3, {0x01, 0xbf, 0x07}},
            {(long)8E6, (long)50E3, {0x03, 0xb4, 0x06}},
            {(long)8E6, (long)40E3, {0x03, 0xbf, 0x07}},
            {(long)8E6, (long)20E3, {0x07, 0xbf, 0x07}},
            {(long)8E6, (long)10E3, {0x0f, 0xbf, 0x07}},
            {(long)8E6, (long)5E3, {0x1f, 0xbf, 0x07}},

            {(long)16E6, (long)1000E3, {0x00, 0xd0, 0x82}},
            {(long)16E6, (long)500E3, {0x00, 0xf0, 0x86}},
            {(long)16E6, (long)250E3, {0x41, 0xf1, 0x85}},
            {(long)16E6, (long)200E3, {0x01, 0xfa, 0x87}},
            {(long)16E6, (long)125E3, {0x03, 0xf0, 0x86}},
            {(long)16E6, (long)100E3, {0x03, 0xfa, 0x87}},
            {(long)16E6, (long)80E3, {0x03, 0xff, 0x87}},
            {(long)16E6, (long)50E3, {0x07, 0xfa, 0x87}},
            {(long)16E6, (long)40E3, {0x07, 0xff, 0x87}},
            {(long)16E6, (long)20E3, {0x0f, 0xff, 0x87}},
            {(long)16E6, (long)10E3, {0x1f, 0xff, 0x87}},
            {(long)16E6, (long)5E3, {0x3f, 0xff, 0x87}},
        };

        const uint8_t *cnf = NULL;

        for (unsigned int i = 0; i < (sizeof(CNF_MAPPER) / sizeof(CNF_MAPPER[0])); i++)
        {
            if (CNF_MAPPER[i].clockFrequency == m_clock_freq && CNF_MAPPER[i].baudRate == baudRate)
            {
                cnf = CNF_MAPPER[i].cnf;
                break;
            }
        }

        if (cnf == NULL)
        {
            return -1;
        }

        write_register(REG_CNF1, cnf[0]);
        write_register(REG_CNF2, cnf[1]);
        write_register(REG_CNF3, cnf[2]);
        return 0;
    }

    uint8_t read_register(uint8_t reg)
    {
        SPIWriter writer(m_settings, m_cs_pin);
        writer.transfer(0b0000011);
        writer.transfer(reg);
        uint8_t retval = writer.transfer(0x00); // read data
        return retval;
    }

    void write_register(uint8_t reg, uint8_t value)
    {
        SPIWriter writer(m_settings, m_cs_pin);
        writer.transfer(0b0000010);
        writer.transfer(reg);
        writer.transfer(value); // send data
    }

    void modify_register(uint8_t reg, uint8_t mask, uint8_t val)
    {
        SPIWriter writer(m_settings, m_cs_pin);
        writer.transfer(0b00000101);
        writer.transfer(reg);
        writer.transfer(mask);
        writer.transfer(val);
    }

private:
    int m_cs_pin;
    int m_int_pin;
    long m_clock_freq;
    SPISettings m_settings;

    bool m_tx_packet_begun = false;
    PacketData m_tx_packet_buffer;
};

#endif