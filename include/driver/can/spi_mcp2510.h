/*
 * This driver is based on the mcp251x.c driver from Linux and uses raw pico-sdk SPI access.
 *
 * This is both an SPI and a CAN driver
 */
#include "platform/platform.h"
#include "driver/can/can_timing.h"
#include <hardware/spi.h>
#include <pico/mutex.h>
#include <hardware/gpio.h>

#define INSTRUCTION_WRITE 0x02
#define INSTRUCTION_READ 0x03
#define INSTRUCTION_BIT_MODIFY 0x05
#define INSTRUCTION_LOAD_TXB(n) (0x40 + 2 * (n))
#define INSTRUCTION_READ_RXB(n) (((n) == 0) ? 0x90 : 0x94)
#define INSTRUCTION_RESET 0xC0
#define RTS_TXB0 0x01
#define RTS_TXB1 0x02
#define RTS_TXB2 0x04
#define INSTRUCTION_RTS(n) (0x80 | ((n)&0x07))

/* MPC251x registers */
#define BFPCTRL 0x0c
#define BFPCTRL_B0BFM BIT(0)
#define BFPCTRL_B1BFM BIT(1)
#define BFPCTRL_BFM(n) (BFPCTRL_B0BFM << (n))
#define BFPCTRL_BFM_MASK GENMASK(1, 0)
#define BFPCTRL_B0BFE BIT(2)
#define BFPCTRL_B1BFE BIT(3)
#define BFPCTRL_BFE(n) (BFPCTRL_B0BFE << (n))
#define BFPCTRL_BFE_MASK GENMASK(3, 2)
#define BFPCTRL_B0BFS BIT(4)
#define BFPCTRL_B1BFS BIT(5)
#define BFPCTRL_BFS(n) (BFPCTRL_B0BFS << (n))
#define BFPCTRL_BFS_MASK GENMASK(5, 4)
#define TXRTSCTRL 0x0d
#define TXRTSCTRL_B0RTSM BIT(0)
#define TXRTSCTRL_B1RTSM BIT(1)
#define TXRTSCTRL_B2RTSM BIT(2)
#define TXRTSCTRL_RTSM(n) (TXRTSCTRL_B0RTSM << (n))
#define TXRTSCTRL_RTSM_MASK GENMASK(2, 0)
#define TXRTSCTRL_B0RTS BIT(3)
#define TXRTSCTRL_B1RTS BIT(4)
#define TXRTSCTRL_B2RTS BIT(5)
#define TXRTSCTRL_RTS(n) (TXRTSCTRL_B0RTS << (n))
#define TXRTSCTRL_RTS_MASK GENMASK(5, 3)
#define CANSTAT 0x0e
#define CANCTRL 0x0f
#define CANCTRL_REQOP_MASK 0xe0
#define CANCTRL_REQOP_CONF 0x80
#define CANCTRL_REQOP_LISTEN_ONLY 0x60
#define CANCTRL_REQOP_LOOPBACK 0x40
#define CANCTRL_REQOP_SLEEP 0x20
#define CANCTRL_REQOP_NORMAL 0x00
#define CANCTRL_OSM 0x08
#define CANCTRL_ABAT 0x10
#define TEC 0x1c
#define REC 0x1d
#define CNF1 0x2a
#define CNF1_SJW_SHIFT 6
#define CNF2 0x29
#define CNF2_BTLMODE 0x80
#define CNF2_SAM 0x40
#define CNF2_PS1_SHIFT 3
#define CNF3 0x28
#define CNF3_SOF 0x08
#define CNF3_WAKFIL 0x04
#define CNF3_PHSEG2_MASK 0x07
#define CANINTE 0x2b
#define CANINTE_MERRE 0x80
#define CANINTE_WAKIE 0x40
#define CANINTE_ERRIE 0x20
#define CANINTE_TX2IE 0x10
#define CANINTE_TX1IE 0x08
#define CANINTE_TX0IE 0x04
#define CANINTE_RX1IE 0x02
#define CANINTE_RX0IE 0x01
#define CANINTF 0x2c
#define CANINTF_MERRF 0x80
#define CANINTF_WAKIF 0x40
#define CANINTF_ERRIF 0x20
#define CANINTF_TX2IF 0x10
#define CANINTF_TX1IF 0x08
#define CANINTF_TX0IF 0x04
#define CANINTF_RX1IF 0x02
#define CANINTF_RX0IF 0x01
#define CANINTF_RX (CANINTF_RX0IF | CANINTF_RX1IF)
#define CANINTF_TX (CANINTF_TX2IF | CANINTF_TX1IF | CANINTF_TX0IF)
#define CANINTF_ERR (CANINTF_ERRIF)
#define EFLG 0x2d
#define EFLG_EWARN 0x01
#define EFLG_RXWAR 0x02
#define EFLG_TXWAR 0x04
#define EFLG_RXEP 0x08
#define EFLG_TXEP 0x10
#define EFLG_TXBO 0x20
#define EFLG_RX0OVR 0x40
#define EFLG_RX1OVR 0x80
#define TXBCTRL(n) (((n)*0x10) + 0x30 + TXBCTRL_OFF)
#define TXBCTRL_ABTF 0x40
#define TXBCTRL_MLOA 0x20
#define TXBCTRL_TXERR 0x10
#define TXBCTRL_TXREQ 0x08
#define TXBSIDH(n) (((n)*0x10) + 0x30 + TXBSIDH_OFF)
#define SIDH_SHIFT 3
#define TXBSIDL(n) (((n)*0x10) + 0x30 + TXBSIDL_OFF)
#define SIDL_SID_MASK 7
#define SIDL_SID_SHIFT 5
#define SIDL_EXIDE_SHIFT 3
#define SIDL_EID_SHIFT 16
#define SIDL_EID_MASK 3
#define TXBEID8(n) (((n)*0x10) + 0x30 + TXBEID8_OFF)
#define TXBEID0(n) (((n)*0x10) + 0x30 + TXBEID0_OFF)
#define TXBDLC(n) (((n)*0x10) + 0x30 + TXBDLC_OFF)
#define DLC_RTR_SHIFT 6
#define TXBCTRL_OFF 0
#define TXBSIDH_OFF 1
#define TXBSIDL_OFF 2
#define TXBEID8_OFF 3
#define TXBEID0_OFF 4
#define TXBDLC_OFF 5
#define TXBDAT_OFF 6
#define RXBCTRL(n) (((n)*0x10) + 0x60 + RXBCTRL_OFF)
#define RXBCTRL_BUKT 0x04
#define RXBCTRL_RXM0 0x20
#define RXBCTRL_RXM1 0x40
#define RXBSIDH(n) (((n)*0x10) + 0x60 + RXBSIDH_OFF)
#define RXBSIDH_SHIFT 3
#define RXBSIDL(n) (((n)*0x10) + 0x60 + RXBSIDL_OFF)
#define RXBSIDL_IDE 0x08
#define RXBSIDL_SRR 0x10
#define RXBSIDL_EID 3
#define RXBSIDL_SHIFT 5
#define RXBEID8(n) (((n)*0x10) + 0x60 + RXBEID8_OFF)
#define RXBEID0(n) (((n)*0x10) + 0x60 + RXBEID0_OFF)
#define RXBDLC(n) (((n)*0x10) + 0x60 + RXBDLC_OFF)
#define RXBDLC_LEN_MASK 0x0f
#define RXBDLC_RTR 0x40
#define RXBCTRL_OFF 0
#define RXBSIDH_OFF 1
#define RXBSIDL_OFF 2
#define RXBEID8_OFF 3
#define RXBEID0_OFF 4
#define RXBDLC_OFF 5
#define RXBDAT_OFF 6
#define RXFSID(n) ((n < 3) ? 0 : 4)
#define RXFSIDH(n) ((n)*4 + RXFSID(n))
#define RXFSIDL(n) ((n)*4 + 1 + RXFSID(n))
#define RXFEID8(n) ((n)*4 + 2 + RXFSID(n))
#define RXFEID0(n) ((n)*4 + 3 + RXFSID(n))
#define RXMSIDH(n) ((n)*4 + 0x20)
#define RXMSIDL(n) ((n)*4 + 0x21)
#define RXMEID8(n) ((n)*4 + 0x22)
#define RXMEID0(n) ((n)*4 + 0x23)

#define SPI_TRANSFER_BUF_LEN (6 + 8)

#define ERR_SPI_WRITE_FAILED -1

/**
 * Private instance data for the driver
 */
struct Priv
{
    mutex_t mcp_lock;
    u8 spi_tx_buf[SPI_TRANSFER_BUF_LEN];
    u8 spi_rx_buf[SPI_TRANSFER_BUF_LEN];
    spi_inst_t *spi;
};

static int mcp251x_spi_write(Priv *priv, size_t len)
{
    gpio_put(PLATFORM_PIN_CAN_CS, 0);
    int ret = spi_write_blocking(priv->spi, priv->spi_tx_buf, len);
    gpio_put(PLATFORM_PIN_CAN_CS, 1);
    if (ret != (int)len)
    {
        return ERR_SPI_WRITE_FAILED;
    }
    return 0;
}

static int mcp251x_spi_transfer(Priv *priv, size_t len)
{
    gpio_put(PLATFORM_PIN_CAN_CS, 0);
    int ret = spi_write_read_blocking(priv->spi, priv->spi_tx_buf, priv->spi_rx_buf, len);
    gpio_put(PLATFORM_PIN_CAN_CS, 1);
    return ret;
}

static int mcp251x_read_reg(Priv *priv, u8 reg)
{
    priv->spi_tx_buf[0] = INSTRUCTION_READ;
    priv->spi_tx_buf[1] = reg;

    mcp251x_spi_transfer(priv, 3);
    return priv->spi_rx_buf[2];
}

static void mcp251x_write_reg(Priv *priv, u8 reg, u8 value)
{
    priv->spi_tx_buf[0] = INSTRUCTION_WRITE;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = value;

    mcp251x_spi_write(priv, 3);
}

static void mcp251x_write_bits(Priv *priv, u8 reg, u8 mask, u8 value)
{
    priv->spi_tx_buf[0] = INSTRUCTION_BIT_MODIFY;
    priv->spi_tx_buf[1] = reg;
    priv->spi_tx_buf[2] = mask;
    priv->spi_tx_buf[3] = value;

    mcp251x_spi_write(priv, 4);
}

static u8 mcp251x_read_stat(Priv *priv)
{
    return mcp251x_read_reg(priv, CANSTAT) & CANCTRL_REQOP_MASK;
}

static int mcp251x_reset(Priv *priv)
{
    delay(5); // Wait for oscillator startup

    // send reset
    priv->spi_tx_buf[0] = INSTRUCTION_RESET;
    int ret = mcp251x_spi_write(priv, 1);
    if (ret)
    {
        return ret;
    }
    delay(5); // Wait for oscillator reset

    // wait for reset
    while (mcp251x_read_stat(priv) != CANCTRL_REQOP_CONF)
    {
        platform_set_status(STATUS_CAN_RESETWAIT); // signal waiting for can reset, we cannot proceed without CAN
    }

    return 0;
}

static void mcp251x_can_isr(void *device)
{
}

static int mcp251x_set_bittiming(Priv *priv, int clock_freq, int baudrate)
{
    const u8 *cnf = nullptr;
    for (unsigned int i = 0; i < (sizeof(mcp251x_cnf_mapper)) / sizeof(mcp251x_cnf_mapper[0]); i++)
    {
        if (mcp251x_cnf_mapper[i].clockFrequency == clock_freq && mcp251x_cnf_mapper[i].baudRate == baudrate)
        {
            cnf = mcp251x_cnf_mapper[i].cnf;
            break;
        }
    }

    if (!cnf)
    {
        return STATUS_CAN_INVALIDSPEED;
    }
    mcp251x_write_reg(priv, CNF1, cnf[0]);
    mcp251x_write_reg(priv, CNF2, cnf[1]);
    mcp251x_write_reg(priv, CNF3, cnf[2]);

    return 0;
}

static int mcp251x_set_normal_mode(Priv *priv)
{
    // enable all interrupts
    mcp251x_write_reg(priv, CANINTE, CANINTE_ERRIE | CANINTE_TX2IE | CANINTE_TX1IE | CANINTE_TX0IE | CANINTE_RX1IE | CANINTE_RX0IE);

    /* Put device into normal mode */
    mcp251x_write_reg(priv, CANCTRL, CANCTRL_REQOP_NORMAL);

    // set normalmode
    while (mcp251x_read_stat(priv) != CANCTRL_REQOP_NORMAL)
    {
        platform_set_status(STATUS_CAN_MODESETWAIT); // signal waiting for can reset, we cannot proceed without CAN
    }

    return 0;
}

static Priv *mcp251x_platform_init(int clock_freq, int baudrate)
{
    // init spi
    _spi_init(PLATFORM_CAN_SPI, PLATFORM_CAN_SPI_BAUD);
    gpio_set_function(PLATFORM_PIN_SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PLATFORM_PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PLATFORM_PIN_SPI_SCK, GPIO_FUNC_SPI);
    // todo: spi set format

    // init cs
    _gpio_init(PLATFORM_PIN_CAN_CS);
    gpio_set_dir(PLATFORM_PIN_CAN_CS, GPIO_OUT);
    gpio_put(PLATFORM_PIN_CAN_CS, 1); // CS is active low

    Priv *instance = new Priv;
    mutex_init(&instance->mcp_lock);
    memset(instance->spi_tx_buf, 0, SPI_TRANSFER_BUF_LEN);
    memset(instance->spi_rx_buf, 0, SPI_TRANSFER_BUF_LEN);

    mutex_enter_blocking(&instance->mcp_lock);

    attachInterruptParam(PLATFORM_PIN_CAN_INT, mcp251x_can_isr, PinStatus::FALLING, instance);

    // reset device
    int ret = mcp251x_reset(instance);
    if (ret != 0)
    {
        platform_set_status(STATUS_CAN_RESETFAILED);
        goto retnull;
    }

    ret = mcp251x_set_bittiming(instance, clock_freq, baudrate);
    if (ret)
    {
        platform_set_status(ret);
        goto retnull;
    }

    mcp251x_write_reg(instance, RXBCTRL(0), RXBCTRL_BUKT | RXBCTRL_RXM0 | RXBCTRL_RXM1);
    mcp251x_write_reg(instance, RXBCTRL(1), RXBCTRL_RXM0 | RXBCTRL_RXM1);

    // Skip null return
    goto normalret;

retnull:
    delete instance;
    instance = nullptr;
    detachInterrupt(PLATFORM_PIN_CAN_INT);

normalret:
    mutex_exit(&instance->mcp_lock);
    return instance;
}
