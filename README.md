# MyBLEGamepad

## 功能介绍：

- #### 可以模拟xbox蓝牙手柄，SwitchPro蓝牙手柄，DualSense蓝牙手柄；

- #### 所有手柄都支持振动；
- #### switchPro手柄跟dualSense手柄支持陀螺仪体感操作；
- #### 除了switchPro手柄没有线性扳机，其他两个都有线性扳机；
- #### 支持绑定多台主机，3种模式可以同时绑定同一主机，使用时随意切换，无需重新绑定；
- #### 可以手动设置摇杆偏移，摇杆死区，陀螺仪自动校准；
- #### 正常刷新率在60Hz左右，可以开启高性能模式，刷新率可以达到120Hz；
- #### 支持win10平台：yuzu cemu dolphin模拟器跟steam都可以正常识别；
- #### 支持Android平台：我的小米10s装的DraStic模拟器也可以正常识别；




## 使用说明：

- ## 开关机及充电：

    - 开机：关机状态下，按一下xbox键，灯亮开机；
        * 开机蓝牙自动广播，LED灯有滚动效果，30秒没连接上自动关机；
        * 连接后，LED显示对应的手柄指示灯。

    - 关机：开机状态下，长按xbox键3秒，灯灭关机。

    - 充电：手柄插入usb电源自动充电，可以边充电边使用。
        * 关机状态下充电：LED显示当前电量，充电中会闪烁，充满时常亮。
        * 开机状态下充电：LED不会显示电量，只显示手柄模式，按一下xbox键切换到电量显示，3秒后自动切回显示当前手柄。

- ## 手柄模式：

    - 关机状态下按下：X + XBOX：
        * 进入xbox手柄模式，右上角LED指示灯亮起并播放广播灯效。

    - 关机状态下按下：A + XBOX：
        * 进入SwitchPro手柄模式，右下角LED指示灯亮起并播放广播灯效。

    - 关机状态下按下：Y + XBOX：
        * 进入DualSense手柄模式，左上角LED指示灯亮起并播放广播灯效。

    - 关机状态下按下：XBOX：
        * 进入上一次使用的的手柄模式，相应指示灯亮起。

    - 关机状态下按下：（L + A 或 X 或 Y + XBOX ）或（L + XBOX）：
        * 手柄会振动提示一下，进入对应手柄的高性能输出模式；
        * 高性能模式耗电增加，延迟降低到6~7ms左右。
    
    - 关机状态下按下：（START + A 或 X 或 Y + XBOX）或（START + XBOX）：
        * 清除蓝牙绑定，并进入对应手柄的模式。

    - 关机状态下按下：B + XBOX:
        * 进入校准模式，LED显示跑马灯效果。

    - 断电状态下：按下SELECT并用USB接通电脑：
        * 进入下载模式，该模式只再芯片首次烧录会用到。
    

- ## 校准模式：

        * 校准模式需要使用串口进行通信，所以需要用usb连接电脑；
        * 还有打开串口调试助手，串口波特率为115200；
        * 其他详细指令看串口输出的帮助说明。

- ## 绑定与删除绑定：

    * 3个手柄的名称分别为：MyXboxGamepad、MyProGamepad、MyDSGamepad；可以一起绑定。

    * 删除绑定：开机时，先按住start键，再按照上面提到的手柄模式开机，将自动删除对应手柄的绑定，
        * 此时手柄会处于广播状态。
        * （注意：Win10电脑绑定的信息不会自动删掉，需要手动删除，不然电脑会自动连接手柄，手柄自动断开连接，反复连接断开。）

- ## 该项目目录介绍：
    * 3D：原PCB描图，Solidworks文件，用处不大。
    * bootloader_components：bootloader代码
    * components：组件代码
    * main：该项目源码
    * pcb：
        * .epro文件：力创EDA专业版工程文件。
        * .pdf文件：eda导出的pdf，方便查阅。
    * prebuild：该项目预先编译好的bin文件，可以直接烧录，注意我用的芯片模组是：ESP32-S3-WROOM-1-N8R2，其他模组可能要自己修改编译一下了。

- ## 制作相关：
    * 我是直接网购的一个20块钱左右的有线带振动的xbox手柄来进行改装的。
    * 然后重新配着这个手柄，照着它的PCB轮廓重新描画，再在嘉立创免费打样。
    * pcb文件夹里面有该pcb相关的工程文件，pcb也调整到嘉立创可以免费打样的尺寸100*100mm；pcb文件夹里面的pdf有部分原件不焊的说明。
    * 原手柄的部分电子原件需要拆下来，省去另外购买：
        * 左右肩键：拆下来，引脚掰直，可以用。
        * 左右扳机电位计：拆下来，同样可以用。
         （注意：该手柄扳机键前段有一段很长的空行程，可以拿东西垫一下那个摇杆滑槽，有条件的可以直接在滑槽那里拧入一颗调节螺丝，这样可以直接调整扳机键的行程。）
        * 左右摇杆：它这个左右摇杆很辣鸡，线性非常差，我是另外花了2块钱买了2个摇杆替换的，线性度还行，就是尺寸有点大，所有PCB里面摇杆的孔位我是改过的。其实用回原来的摇杆然后替换掉里面的电位计，也是可以的，PCB的孔位就要改一下了。
        * 振动马达：拆下来。
    * 其他那些电容电阻的封装我都是看我有什么就用什么的，没什么讲究，盖上外壳可能会跟背盖上的筋有点干涉，要削一下。
    * 充电管理芯片：TP4056
    * 稳压芯片：AP2112K-3.3TR
    * 陀螺仪：MPU6050
    * 磁力计：QMC5883L，我用电烙铁焊了两个钟都焊不上，放弃，也用不上，有也只是多个点缀，目前那些带体感看的手柄它们传输的数据也没磁力计。
    * 电池：手柄内部空间我测量了下，估计最多可以塞个300~400mAh的电池，太小了，ESP32耗电其实也挺高的，正常可以到70~80mA，两个振动马达1个100mA左右，玩不了多久就没电，所以我是选择将电池贴到手柄壳外面，想贴多大贴多大，我买了个800mAh，感觉还是小了点，可以搞个1600mAh的。
    * 那些硅胶键的焊盘我是加了些焊锡的，不然太平了，接触不太好，但是也别加太厚，太厚按键按不下去，没手感。

- ## 烧录相关：
    * 使用idf烧录：
        * 对于安装了Espressif开发工具的，可以直接用idf.py自行进行编译烧录。
        * 按住手柄SELECT按键，再用USB插入电脑，自动进入下载模式，然后按正常idf编译烧录方式烧录了。
    * 使用烧录工具烧录：
        * https://www.espressif.com.cn/zh-hans/support/download/other-tools
        * 去上面的路径下载Flash下载工具。
        * 打开烧录工具，ChipType选ESP32-S3，WorkMode选Develop，LoadMode选UART。
        * 直接使用该项目编译好的bin文件，在prebuild目录下，填入路径、偏移地址、打勾：
            * bootloader.bin 偏移地址 0x0
            * partition-table.bin 偏移地址 0x8000
            * main.bin 偏移地址 0x10000
        * SPI Speed：80M
        * SPI MODE：DIO
        * COM：选你的串口号，如：COM11
        * BAUD：115200
        * 最后点start，开始烧录，烧完拔掉USB重新上电就可以了。

- ## 视频演示：
    * https://www.bilibili.com/video/BV1YaG6e4Eqe

- ## 主要材料及价格：
    |   材料    |   数量    |   价格￥    |
    |:-:|:-:|-:|
    |山寨xbox有线手柄带振动|1|20.06|
    |ESP32s3-WROOM-1N8R2|1|17.80|
    |800mAh软包锂电池|1|10.60|
    |TP4056|1|0.38|
    |MPU6050|1|4.50|
    |AP2112K-3.3TR|1|0.35|
    |Type-c|1|0.32|
    |1N5819W|19|1.34|
    |SI2305|2|0.06|
    |SI2302|3|0.06|
    |LED|4|0.04|
    |摇杆|2|1.95|
    |PCB嘉立创白嫖|1|0.00|
    |100k电阻|8||
    |10k电阻|5||
    |5.1k电阻|2||
    |1k电阻|2||
    |2欧电阻(可以尝试去掉)|2||
    |0.5欧电阻|1||
    |2.2nF电容|1||
    |10nF电容|1||
    |100nF电容|4||
    |0.2uF电容|2||
    |1uF电容|3||
    |10uF电容|6||
    |22uF电容|1||

总计60块钱左右。

- ## 可能潜在的问题：

    * #### 所有手柄模式都没有在对应的主机测试过，所以不保证可以在主机运行。


- ## 参考资料：
    * https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering.git
    * https://github.com/mumumusuc/libjoycon.git
    * https://github.com/EasyConNS/BlueCon-esp32.git
    * https://controllers.fandom.com/wiki/Sony_DualSense
    * https://github.com/libsdl-org/SDL.git