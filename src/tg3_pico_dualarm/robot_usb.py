"""PICO USB input on robot Ubuntu. Only this project's app/reverse is touched."""
import os,re,shutil,subprocess
from . import __version__

PACKAGE='edu.scu.tg3.wireless'

class RobotHeadsetUSB:
    def __init__(self,adb=None,*,run=subprocess.run):
        self.adb=adb or shutil.which('adb')
        if not self.adb:raise RuntimeError('robot_adb_missing_install_Ubuntu_adb_package')
        self.run=run;self.serial=None;self.target=None
        self.identity=None

    def command(self,*args):
        result=self.run([self.adb,*args],stdin=subprocess.DEVNULL,capture_output=True,text=True,timeout=8)
        if result.returncode:raise RuntimeError('robot_usb_adb_command_failed:'+str(args[0])+':'+result.stderr.strip()[:300])
        return result.stdout.strip()

    def identify(self):
        entries=[line.split() for line in self.command('devices').splitlines()[1:] if line.strip()]
        if any(len(v)>=2 and v[1]=='unauthorized' for v in entries):
            raise RuntimeError('请在头显允许此机器人电脑的USB调试授权，再重新启动')
        active=[v[0] for v in entries if len(v)==2 and v[1]=='device']
        if len(active)!=1:raise RuntimeError('机器人USB需恰好一台已授权PICO；请检查头显数据线')
        serial=active[0]
        if not re.fullmatch(r'[A-Za-z0-9._:-]{1,128}',serial):raise RuntimeError('invalid_robot_usb_device_serial')
        def prop(name):return self.command('-s',serial,'shell','getprop',name)
        identity=' '.join(prop(name) for name in ('ro.product.manufacturer','ro.product.brand','ro.product.model'))
        if 'pico' not in identity.lower() and 'a9210' not in identity.lower():raise RuntimeError('robot_usb_device_is_not_PICO')
        if not self.command('-s',serial,'shell','pm','path',PACKAGE).startswith('package:'):
            raise RuntimeError('头显未安装本项目App，请先在电脑完成首次安装')
        package=self.command('-s',serial,'shell','dumpsys','package',PACKAGE)
        if not re.search(r'\bversionName='+re.escape(__version__)+r'(?:\s|$)',package):
            raise RuntimeError('头显App版本不匹配，请安装交付包中的本项目APK')
        self.serial=serial
        self.identity=dict(model=identity,transport='robot_usb',device_authorized=True,
            app_version=__version__,operator_uid=os.geteuid() if hasattr(os,'geteuid') else None)
        return self.identity.copy()

    def check_available(self):
        identity=self.identify()
        mappings=self.command('-s',self.serial,'reverse','--list')
        if any(len(line.split())>=2 and line.split()[-2]=='tcp:8877' for line in mappings.splitlines()):
            raise RuntimeError('头显8877端口已有转发，请先停止之前的本项目会话；不会覆盖现有转发')
        return identity

    def prepare(self,port):
        if type(port) is not int or not 1<=port<=65535:raise ValueError('invalid_robot_usb_receiver_port')
        identity=self.check_available()
        self.command('-s',self.serial,'reverse','--no-rebind','tcp:8877','tcp:'+str(port))
        self.target='tcp:'+str(port)
        try:
            self.command('-s',self.serial,'shell','am','force-stop',PACKAGE)
            self.command('-s',self.serial,'shell','am','start','-n',PACKAGE+'/.PairActivity')
        except BaseException:
            self.close();raise
        return identity

    def close(self):
        if not self.serial or not self.target:return
        try:
            mappings=self.command('-s',self.serial,'reverse','--list')
            if any(line.split()[-2:]==['tcp:8877',self.target] for line in mappings.splitlines()):
                self.command('-s',self.serial,'reverse','--remove','tcp:8877')
        except (RuntimeError,subprocess.TimeoutExpired,OSError):pass
        self.target=None
