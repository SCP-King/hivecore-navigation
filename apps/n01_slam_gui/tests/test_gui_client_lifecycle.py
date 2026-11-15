import asyncio,sys
from pathlib import Path
from types import SimpleNamespace as S
from unittest.mock import Mock
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from gui_client_lifecycle import ClientUI,ViewCache


def test_deleted_view_cancels_timers_and_reconnect_gets_new_widgets_and_cache():
    client=S(id='old',_deleted=False)
    owner=ClientUI(lambda:client);old=owner.attach(client)
    owner['log']=Mock();cache=ViewCache(owner,'map',{'key':None});cache['key']='old pixels'
    timer=Mock();old.timers.append(timer)
    owner.release(old);client._deleted=True
    assert not owner.available;timer.cancel.assert_called_once()
    with pytest.raises(asyncio.CancelledError):owner['log']
    client=S(id='new',_deleted=False);owner.attach(client)
    assert 'log' not in owner and cache['key'] is None
    owner['log']='new';assert owner['log']=='new'


def test_deleted_async_callback_cannot_write_into_reconnected_page():
    async def scenario():
        current=[S(id='old',_deleted=False)]
        owner=ClientUI(lambda:current[0]);old=owner.attach(current[0]);owner['log']=Mock()
        started=asyncio.Event();release=asyncio.Event();writes=[]
        async def pending():
            widget=owner['log'];started.set()
            await release.wait()
            widget.push('late');writes.append('old write')
        task=asyncio.create_task(pending());await started.wait()
        owner.release(old)
        current[0]=S(id='new',_deleted=False);owner.attach(current[0]);owner['log']=Mock()
        release.set()
        with pytest.raises(asyncio.CancelledError):await task
        assert not writes
        owner['log'].push.assert_not_called()
    asyncio.run(scenario())


def test_clients_keep_independent_widgets_and_map_click_coordinates():
    current=[S(id='a',_deleted=False)];owner=ClientUI(lambda:current[0]);a=owner.attach(current[0])
    cache=ViewCache(owner,'view');owner['label']='a';cache['x0']=1
    current[0]=S(id='b',_deleted=False);b=owner.attach(current[0]);owner['label']='b';cache['x0']=20
    current[0]=a.client
    assert owner['label']=='a' and cache['x0']==1
    owner.release(a);current[0]=b.client
    assert owner['label']=='b' and cache['x0']==20
