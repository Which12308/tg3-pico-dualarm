"""Display only actual URDF link-frame coordinates supplied by the solver."""
import math
from numbers import Real
import tkinter as tk
from tkinter import ttk

from tg3_pico_dualarm.model import DualArmModel,BASE_FRAME


def checked_chains(model,links):
    if not isinstance(links,dict):return {}
    result={}
    for side,arm in model.arms.items():
        supplied=links.get(side)
        if not isinstance(supplied,dict):continue
        names=[BASE_FRAME]+[joint.child for joint in arm.joints]+[arm.tip_frame]
        points=[]
        for name in names:
            value=supplied.get(name)
            if (not isinstance(value,(list,tuple)) or len(value)!=3 or
                any(isinstance(v,bool) or not isinstance(v,Real) or not math.isfinite(v) for v in value)):
                points=[];break
            points.append(tuple(float(v) for v in value))
        if points:result[side]=points
    return result


def project(point,view):
    x,y,z=point
    if view=='正视 YZ':return y,z
    if view=='侧视 XZ':return x,z
    if view=='俯视 XY':return y,x
    azimuth=math.radians(45);elevation=math.radians(25)
    return -math.sin(azimuth)*x+math.cos(azimuth)*y, -math.sin(elevation)*math.cos(azimuth)*x-math.sin(elevation)*math.sin(azimuth)*y+math.cos(elevation)*z


class ArmPreview(ttk.Frame):
    def __init__(self,parent):
        super().__init__(parent)
        self.model=DualArmModel();self.chains={};self.message='等待求解器的模型坐标'
        self.view=tk.StringVar(value='三维示意')
        header=ttk.Frame(self);header.pack(fill='x',padx=8,pady=6)
        ttk.Label(header,text='URDF 运动学模型 · 现场实物请直接观察',font=('Microsoft YaHei UI',11)).pack(side='left')
        selection=ttk.Combobox(header,textvariable=self.view,values=('三维示意','正视 YZ','侧视 XZ','俯视 XY'),state='readonly',width=13)
        selection.pack(side='right');selection.bind('<<ComboboxSelected>>',lambda _event:self.redraw())
        self.canvas=tk.Canvas(self,background='#10202c',highlightthickness=0,height=320)
        self.canvas.pack(fill='both',expand=True,padx=8,pady=5)
        self.canvas.bind('<Configure>',lambda _event:self.redraw())
        self.source=tk.StringVar(value='数据来源：等待原生求解；画面始终是模型')
        ttk.Label(self,textvariable=self.source).pack(anchor='w',padx=8)
        hands=ttk.Frame(self);hands.pack(fill='x',padx=8,pady=6)
        self.hand_values={}
        for column,(side,label) in enumerate((('left','左手闭合指令'),('right','右手闭合指令'))):
            ttk.Label(hands,text=label).grid(row=0,column=column*2,padx=5)
            value=tk.DoubleVar(value=0.);self.hand_values[side]=value
            ttk.Progressbar(hands,variable=value,maximum=100,length=180).grid(row=0,column=column*2+1,padx=5,sticky='ew')
        hands.columnconfigure(1,weight=1);hands.columnconfigure(3,weight=1)

    def update_control(self,control):
        control=control if isinstance(control,dict) else {}
        self.chains=checked_chains(self.model,control.get('model_links'))
        state=str(control.get('state','等待'))
        self.message='当前状态：'+state if not self.chains else ''
        self.source.set('数据来源：'+('真机控制目标的 URDF 计算' if state=='hardware' else '模型验证求解结果')+
            '；基坐标 '+BASE_FRAME+'；画面始终是模型')
        closure=control.get('hands',{}).get('closure',{})
        for side,value in self.hand_values.items():
            raw=closure.get(side,0.)
            value.set(100*max(0.,min(1.,raw)) if isinstance(raw,Real) and not isinstance(raw,bool) and math.isfinite(raw) else 0.)
        self.redraw()

    def redraw(self):
        canvas=self.canvas;canvas.delete('all')
        width=max(200,canvas.winfo_width());height=max(180,canvas.winfo_height())
        canvas.create_text(width/2,18,text='模型骨架（不是相机画面）',fill='#c7d8e3',font=('Microsoft YaHei UI',11))
        if not self.chains:
            canvas.create_text(width/2,height/2,text=self.message,fill='#9cb6c8');return
        view=self.view.get()
        axes=[((.15,0,0),'X 前','#ef6d70'),((0,.15,0),'Y 左','#72d798'),((0,0,.15),'Z 上','#7ab3ff')]
        projected=[project(point,view) for values in self.chains.values() for point in values]
        projected+=[project((0,0,0),view)]+[project(point,view) for point,_,_ in axes]
        low_u=min(v[0] for v in projected);high_u=max(v[0] for v in projected)
        low_v=min(v[1] for v in projected);high_v=max(v[1] for v in projected)
        scale=min((width-120)/max(.2,high_u-low_u),(height-100)/max(.2,high_v-low_v))
        center_u=(low_u+high_u)/2;center_v=(low_v+high_v)/2
        def screen(point):
            u,v=project(point,view);return width/2+(u-center_u)*scale,height/2+15-(v-center_v)*scale
        origin=screen((0,0,0))
        for point,label,color in axes:
            end=screen(point);canvas.create_line(*origin,*end,fill=color,width=2,arrow='last')
            canvas.create_text(end[0]+12,end[1]-8,text=label,fill=color)
        for side,values in self.chains.items():
            color='#53c3ed' if side=='left' else '#ffb465'
            coordinates=[coordinate for point in values for coordinate in screen(point)]
            canvas.create_line(*coordinates,fill=color,width=5)
            for point in values:
                x,y=screen(point);canvas.create_oval(x-3,y-3,x+3,y+3,fill=color,outline='')
            x,y=screen(values[-1]);canvas.create_text(x+30,y,text=('左' if side=='left' else '右')+' TCP',fill=color)
