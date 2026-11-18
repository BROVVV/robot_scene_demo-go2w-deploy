import pytest

from app.live_robot.search_event import make_event, SEARCH_FINISHED, EventIdAllocator
from app.live_robot.search_state_store import SearchStateStore
from app.manual_web_demo.control_ownership import ControlOwner
from app.manual_web_demo.search_session_service import SearchSessionService


@pytest.mark.parametrize('reason',['PLANNING_ERROR','MAP_UNAVAILABLE','BACKEND_FAILURE',
    'PERCEPTION_FAILURE','TIMEOUT'])
def test_store_and_service_report_the_same_terminal_failure(tmp_path,reason):
    store=SearchStateStore()
    store.apply(make_event(allocator=EventIdAllocator(),session_id='test',event_type=SEARCH_FINISHED,payload={'result':reason}))
    assert store.snapshot()['status']=='FAILED'
    service=SearchSessionService(owner=ControlOwner(),session_dir=str(tmp_path/'sessions'))
    try:
        service._apply_session_result({'result':reason,'finish_reason':reason,'exit_code':0})
        assert service.state_snapshot()['status']=='FAILED'
        assert service._status=='FAILED'
    finally:service.shutdown()
