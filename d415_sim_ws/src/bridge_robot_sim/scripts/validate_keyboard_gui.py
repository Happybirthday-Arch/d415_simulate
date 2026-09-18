#!/usr/bin/env python3
"""Exercise actual Tk key events against Gazebo; capture the assembled GUI."""
import json,os,re,signal,subprocess,time
from pathlib import Path
import rclpy
from keyboard_drive import Keyboard
from PyQt5.QtGui import QGuiApplication
repo=Path(__file__).resolve().parents[4];out=repo/'gazebo_robot/validation/integration/gui';out.mkdir(parents=True,exist_ok=True)
os.environ['IGN_PARTITION']=f'robot_gui_test_{os.getpid()}';os.environ['ROS_DOMAIN_ID']=str(70+os.getpid()%100)
log=(out/'launch.log').open('w')
process=subprocess.Popen([str(repo/'d415_sim_ws/run_robot.sh'),'world:=bridge_robot','keyboard:=false','record_mode:=off'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();ui=Keyboard();qt=QGuiApplication([]);report={}
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        ui.root.update();time.sleep(.01)
        if process.poll() is not None:raise RuntimeError('Launch exited')
def key(name,down):
    if down:
        ui.root.focus_force();pump(.1)
    ui.root.event_generate('<KeyPress>' if down else '<KeyRelease>',keysym=name);ui.root.update()
    print('key',name,down,'held',ui.keys,'focus',ui.root.focus_get(),flush=True)
def until(predicate,timeout=30):
    deadline=time.monotonic()+timeout
    while not predicate():
        pump(.1)
        if time.monotonic()>deadline:raise RuntimeError(f'GUI target timeout; keys={ui.keys}, feedback={ui.feedback}')
try:
    deadline=time.monotonic()+90
    while ui.feedback is None:
        pump(.2)
        if time.monotonic()>deadline:raise RuntimeError('No drive feedback')
    # Wait for the Gazebo and RViz windows to finish opening before taking keyboard focus.
    pump(8);ui.root.focus_force();pump(.5);until(lambda:ui.feedback.z==0)
    key('w',True);until(lambda:ui.feedback.x>14.9);report['w_target_rpm']=ui.feedback.x
    key('a',True);until(lambda:ui.feedback.y>13.9);report['wa_target_deg']=ui.feedback.y
    key('a',False);key('w',False);until(lambda:abs(ui.feedback.x)<.01);report['released_rpm']=ui.feedback.x
    key('w',True);until(lambda:ui.feedback.x>5);ui.root.withdraw();until(lambda:abs(ui.feedback.x)<.01);report['focus_loss_rpm']=ui.feedback.x
    ui.root.deiconify();ui.root.focus_force();pump(.5);key('w',False)
    key('space',True);key('space',False);key('w',True);pump(.5);report['space_locked']=ui.locked and ui.feedback.z==1
    key('w',False);key('Return',True);key('Return',False);pump(.5);report['enter_unlocked']=not ui.locked
    windows=subprocess.check_output(['xwininfo','-root','-tree'],text=True)
    import ctypes
    x11=ctypes.CDLL('libX11.so.6');x11.XOpenDisplay.restype=ctypes.c_void_p
    x11.XOpenDisplay.argtypes=[ctypes.c_char_p];display=x11.XOpenDisplay(None)
    x11.XMoveResizeWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_int,ctypes.c_int,ctypes.c_uint,ctypes.c_uint]
    x11.XRaiseWindow.argtypes=[ctypes.c_void_p,ctypes.c_ulong];x11.XFlush.argtypes=[ctypes.c_void_p]
    for label in ('gazebo','rviz'):
        candidates=[]
        for line in windows.splitlines():
            match=re.search(r'(0x[0-9a-f]+) "([^"]*)".*? (\d+)x(\d+)[+-]',line)
            if match:
                ident,title,width,height=match.groups()
                if label in title.lower() and int(width)>200 and int(height)>200:
                    candidates.append((int(width)*int(height),int(ident,16)))
        if candidates:
            ident=max(candidates)[1]
            x11.XMoveResizeWindow(display,ident,20,40,1100,750);x11.XRaiseWindow(display,ident);x11.XFlush(display);pump(1)
            pix=qt.primaryScreen().grabWindow(ident);pix.save(str(out/f'{label}.png'))
            report[label+'_screenshot']=True
    ui.root.lift();ui.root.focus_force();pump(.3)
    qt.primaryScreen().grabWindow(ui.root.winfo_id()).save(str(out/'keyboard.png'))
    report['passed']=(report['w_target_rpm']>14.9 and report['wa_target_deg']>13.9 and
                     abs(report['released_rpm'])<.01 and abs(report['focus_loss_rpm'])<.01 and report['space_locked'] and report['enter_unlocked'])
finally:
    ui.close();ui.node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=30)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
    log.close();(out/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
assert report.get('passed')
