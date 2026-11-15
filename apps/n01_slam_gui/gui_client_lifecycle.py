"""Client-owned widgets, caches and callbacks; ROS/process ownership is separate."""
import asyncio
from collections.abc import MutableMapping
from contextvars import ContextVar
from dataclasses import dataclass, field


@dataclass
class View:
    client: object
    widgets: dict = field(default_factory=dict)
    caches: dict = field(default_factory=dict)
    timers: list = field(default_factory=list)
    tasks: set = field(default_factory=set)
    closed: bool = False


class ClientUI(MutableMapping):
    def __init__(self,current_client):
        self.current_client=current_client
        self.views={}
        self.bound=ContextVar('n01_gui_view',default=None)

    def attach(self,client):
        view=View(client)
        self.views[client.id]=view
        self.bound.set(view)
        return view

    def view(self):
        view=self.bound.get()
        try:client=self.current_client()
        except RuntimeError:client=None
        if client is not None and (view is None or view.client.id!=client.id):
            view=self.views.get(client.id)
            if view is not None:self.bound.set(view)
            else:raise asyncio.CancelledError('no GUI client view')
        if view is None or view.closed or getattr(view.client,'_deleted',False):
            raise asyncio.CancelledError('GUI client deleted')
        try:task=asyncio.current_task()
        except RuntimeError:task=None
        if task is not None and task not in view.tasks:
            view.tasks.add(task)
            task.add_done_callback(view.tasks.discard)
        return view

    @property
    def available(self):
        try:self.view();return True
        except asyncio.CancelledError:return False

    def release(self,view):
        if view.closed:return
        view.closed=True
        for timer in view.timers:timer.cancel()
        try:current=asyncio.current_task()
        except RuntimeError:current=None
        for task in tuple(view.tasks):
            if task is not current and not task.done():task.cancel()
        view.widgets.clear();view.caches.clear()
        if self.views.get(view.client.id) is view:self.views.pop(view.client.id)

    def __getitem__(self,key):return self.view().widgets[key]
    def __setitem__(self,key,value):self.view().widgets[key]=value
    def __delitem__(self,key):del self.view().widgets[key]
    def __iter__(self):return iter(self.view().widgets)
    def __len__(self):return len(self.view().widgets)


class ViewCache(MutableMapping):
    def __init__(self,owner,name,defaults=None):
        self.owner=owner;self.name=name;self.defaults=defaults or {}
    def data(self):return self.owner.view().caches.setdefault(self.name,self.defaults.copy())
    def __getitem__(self,key):return self.data()[key]
    def __setitem__(self,key,value):self.data()[key]=value
    def __delitem__(self,key):del self.data()[key]
    def __iter__(self):return iter(self.data())
    def __len__(self):return len(self.data())
