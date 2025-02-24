# ESP32 小智 AI 聊天机器人 （XiaoZhi AI Chatbot）

本项目是对虾哥的小智 AI 聊天机器人工程进行 fork 后得到的，原项目地址为：<https://github.com/78/xiaozhi-esp32> 其中**nulllab_esp32**分支是专门为 ESP32 进行定制开发的，该分支下 ESP32 与各外设的引脚连接信息如下表所示：

| ESP32 引脚编号 | 连接外设 |
| --- | --- |
| 25 |  麦克风（MIC）的 WS 引脚 |
| 26 |  麦克风（MIC）的 SCK 引脚 |
| 27 |  麦克风（MIC）的 DIN (DO) 引脚 |
| 23 | 扬声器（SPAKER）的 DOUT 引脚 |
| 33 | 扬声器（SPAKER）的 BCLK 引脚 |
| 32 | 扬声器（SPAKER）的 LRCK 引脚 |
| 21 | OLED 显示屏的 SDA 引脚 |
| 22 | OLED 显示屏的 SCL 引脚 |
| 34 | 启动按钮（BOOT BUTTON） |
| 35 | 触摸按钮（TOUCH BUTTON） |
| 36 | 语音识别按钮（ASR BUTTON）|
| 4 | 内置 LED 按钮（BUILTIN LED BUTTON） |

## 编译小智

### ESP IDF 5.3.2 环境搭建

如需搭建 ESP IDF 5.3.2 开发环境以编译小智，请参考虾哥的文档：[Windows搭建 ESP IDF 5.3.2开发环境以及编译小智](https://icnynnzcwou8.feishu.cn/wiki/JEYDwTTALi5s2zkGlFGcDiRknXf)，参考文档的前 3 部分内容即可完成环境搭建。

### 下载代码

使用以下命令克隆本项目的代码：

```shell
git clone git@github.com:nulllaborg/xiaozhi-esp32.git -b nulllab_esp32
```

注意：下载代码后，务必将代码切换到 nulllab_esp32 分支，以确保使用的是为 ESP32 定制的代码版本。

### 编译

本项目的 nulllab_esp32 分支已将默认编译选项设置为适配 ESP32，因此可直接使用以下命令进行编译：

```shell
idf.py build
```

在编译过程中，idf.py 脚本会自动处理依赖项的下载和编译，最终生成可烧录的固件文件。

### 烧录

在编译完成后，将 ESP32 开发板通过数据线连接到电脑，然后执行以下烧录命令，等待烧录过程完成：

```shell
idf.py flash
```

以上步骤完成后，ESP32 小智 AI 聊天机器人即可正常工作。

## 更多信息

更多信息，请查看文档《小智 AI 聊天机器人百科全书》<https://ccnphfhqs21z.feishu.cn/wiki/F5krwD16viZoF0kKkvDcrZNYnhb> 。该文档详细介绍了小智 AI 聊天机器人的功能、使用方法、常见问题解答等内容，有助于你更深入地了解和使用该项目。
