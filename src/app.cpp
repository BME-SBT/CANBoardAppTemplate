#include "app.h"

int app_setup()
{
    return 0;
}
u64 counter = 0;
u16 id = 0;
u64 last_sent = 0;
u64 send_delay = 100;

int app_loop()
{
    if (last_sent + 100 < millis())
    {
        last_sent = millis();
        send_delay = 100;

        CAN_Frame frame(id, reinterpret_cast<u8 *>(&counter), 8);
        int sent = PLATFORM_CAN.send(frame);
        if (!sent)
        {
            logf("sent ID=%x, data=%p", id, (void *)counter);
        }
        else if (sent == CAN_QUEUED)
        {
            logf("queued ID=%x, data=%p", id, (void *)counter);
        }
        else
        {
            logf("send error: %x ID=%x, data=%p", sent, id, (void *)counter);
            send_delay = 1000;
        }

        counter += 1;
        id += 1;
        if (id > 0x7ff)
        {
            id = 0;
        }
    }

    return 0;
}