#include <unity.h>
#include <KJO_CAN_Client.h>
#include "fake_can_transport.h"

void setUp() {}
void tearDown() {}

// sendRequest() transmits the correctly-packed frame and enters PENDING.
void test_send_request_transmits_and_enters_pending()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );

    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    TEST_ASSERT_TRUE( client.isRequestPending() );
    TEST_ASSERT_EQUAL_UINT32( CAN_Pack_ID( CAN_NODE_LCMU, CAN_NODE_GSEMU ), transport._last_send_id );
    TEST_ASSERT_EQUAL_size_t( sizeof( CAN_Command_Frame ), transport._last_send_len );

    CAN_Command_Frame sent;
    memcpy( &sent, transport._last_send_data, sizeof( sent ) );
    TEST_ASSERT_EQUAL( CAN_TARE, sent.command );
}

// A matching response (correct destination/source/command) resolves the
// pending request as COMPLETE, and poll() reports it as CAN_POLL_RESPONSE.
void test_matching_response_completes_request()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    CAN_Response_Frame resp;
    resp.command = CAN_TARE;
    resp.status  = true;
    resp.r_val   = 42.0f;
    transport.enqueueFrame( CAN_Pack_ID( CAN_NODE_GSEMU, CAN_NODE_LCMU ),
                             (uint8_t *)&resp, sizeof( resp ) );

    CAN_Command_Frame inbound_out;
    uint8_t           source_out;
    CAN_Poll_Result   result = client.poll( inbound_out, source_out );

    TEST_ASSERT_EQUAL( CAN_POLL_RESPONSE, result );
    TEST_ASSERT_EQUAL( CAN_REQUEST_COMPLETE, client.requestState( 500 ) );
    TEST_ASSERT_EQUAL_FLOAT( 42.0f, client.response().r_val );
}

// A frame addressed to us that ISN'T the awaited response (different
// command, or arrives with no request pending at all) is reported as an
// inbound request for the caller to dispatch -- e.g. GSEMU's Check_CAN()
// receiving an EMU-initiated CAN_SET_FILL_TARGET.
void test_unrelated_frame_reported_as_inbound_request()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    // No request pending.

    CAN_Command_Frame req;
    req.command = CAN_SET_FILL_TARGET;
    req.param   = 12.5f;
    transport.enqueueFrame( CAN_Pack_ID( CAN_NODE_GSEMU, CAN_NODE_EMU ),
                             (uint8_t *)&req, sizeof( req ) );

    CAN_Command_Frame inbound_out;
    uint8_t           source_out;
    CAN_Poll_Result   result = client.poll( inbound_out, source_out );

    TEST_ASSERT_EQUAL( CAN_POLL_INBOUND_REQUEST, result );
    TEST_ASSERT_EQUAL( CAN_SET_FILL_TARGET, inbound_out.command );
    TEST_ASSERT_EQUAL_FLOAT( 12.5f, inbound_out.param );
    TEST_ASSERT_EQUAL_UINT8( CAN_NODE_EMU, source_out );
}

// A frame not addressed to us at all is ignored (CAN_POLL_NOTHING),
// and does not disturb a currently-pending request.
void test_frame_not_addressed_to_us_is_ignored()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    CAN_Response_Frame resp;
    resp.command = CAN_TARE;
    resp.status  = true;
    resp.r_val   = 1.0f;
    // Addressed to EMU, not GSEMU -- e.g. LCMU replying to a different node.
    transport.enqueueFrame( CAN_Pack_ID( CAN_NODE_EMU, CAN_NODE_LCMU ),
                             (uint8_t *)&resp, sizeof( resp ) );

    CAN_Command_Frame inbound_out;
    uint8_t           source_out;
    CAN_Poll_Result   result = client.poll( inbound_out, source_out );

    TEST_ASSERT_EQUAL( CAN_POLL_NOTHING, result );
    TEST_ASSERT_TRUE( client.isRequestPending() );
}

// No response before the caller's timeout -> TIMED_OUT, without any
// frame having arrived.
void test_no_response_before_timeout_reports_timed_out()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    CAN_Request_State state = client.requestState( 0 );   // 0ms timeout: always elapsed
    TEST_ASSERT_EQUAL( CAN_REQUEST_TIMED_OUT, state );
}

// requestState() before either resolution or timeout reports PENDING.
void test_state_pending_before_response_or_timeout()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    TEST_ASSERT_EQUAL( CAN_REQUEST_PENDING, client.requestState( 500 ) );
}

// A response tagged with the wrong command code (stray/unrelated reply)
// is NOT treated as our response -- reported as an inbound request instead
// (mirrors the existing "stray response" guard in today's blocking code).
void test_response_with_wrong_command_not_treated_as_match()
{
    Fake_CAN_Transport transport;
    CAN_Client client( transport, CAN_NODE_GSEMU );
    client.sendRequest( CAN_NODE_LCMU, CAN_TARE, 0.0f );

    CAN_Response_Frame resp;
    resp.command = CAN_REPORT_CURRENT_WEIGHT;   // not what we asked for
    resp.status  = true;
    resp.r_val   = 0.0f;
    transport.enqueueFrame( CAN_Pack_ID( CAN_NODE_GSEMU, CAN_NODE_LCMU ),
                             (uint8_t *)&resp, sizeof( resp ) );

    CAN_Command_Frame inbound_out;
    uint8_t           source_out;
    CAN_Poll_Result   result = client.poll( inbound_out, source_out );

    TEST_ASSERT_EQUAL( CAN_POLL_INBOUND_REQUEST, result );
    TEST_ASSERT_TRUE( client.isRequestPending() );   // still waiting for the real TARE response
}

int main( int argc, char **argv )
{
    UNITY_BEGIN();
    RUN_TEST( test_send_request_transmits_and_enters_pending );
    RUN_TEST( test_matching_response_completes_request );
    RUN_TEST( test_unrelated_frame_reported_as_inbound_request );
    RUN_TEST( test_frame_not_addressed_to_us_is_ignored );
    RUN_TEST( test_no_response_before_timeout_reports_timed_out );
    RUN_TEST( test_state_pending_before_response_or_timeout );
    RUN_TEST( test_response_with_wrong_command_not_treated_as_match );
    return UNITY_END();
}
