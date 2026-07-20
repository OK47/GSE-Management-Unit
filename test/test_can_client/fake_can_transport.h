#ifndef FAKE_CAN_TRANSPORT_H
#define FAKE_CAN_TRANSPORT_H

#include <KJO_CAN_Transport.h>
#include <string.h>

// Test double: a one-slot inbox the test pushes frames into via
// enqueueFrame(), and a record of the last send() call for assertions.
// Mirrors real CAN behavior closely enough for CAN_Client's logic:
// parsePacket() drains exactly one enqueued frame per call.
class Fake_CAN_Transport : public CAN_Transport
{
    public:
        Fake_CAN_Transport() : _has_frame( false ), _send_count( 0 ) {}

        void enqueueFrame( uint32_t id, const uint8_t *data, size_t len )
        {
            _queued_id  = id;
            memcpy( _queued_data, data, len );
            _queued_len = len;
            _has_frame  = true;
        }

        void send( uint32_t id, const uint8_t *data, size_t len ) override
        {
            _last_send_id  = id;
            memcpy( _last_send_data, data, len );
            _last_send_len = len;
            _send_count++;
        }

        int parsePacket() override
        {
            if( !_has_frame ) return 0;
            _has_frame = false;
            return (int)_queued_len;
        }

        uint32_t packetId() override { return _queued_id; }

        void readInto( uint8_t *buf, size_t len ) override
        {
            memcpy( buf, _queued_data, len );
        }

        uint32_t _last_send_id;
        uint8_t  _last_send_data[8];
        size_t   _last_send_len = 0;
        uint32_t _send_count;

    private:
        bool     _has_frame;
        uint32_t _queued_id;
        uint8_t  _queued_data[8];
        size_t   _queued_len;
};

#endif // FAKE_CAN_TRANSPORT_H
