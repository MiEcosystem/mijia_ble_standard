## 支持的硬件平台

- DA1468x
- DA1469x

DA14585硬件请参考 [Dialog 分支](https://github.com/MiEcosystem/mijia_ble_standard/tree/Dialog)

## 使用说明

1. 获取[DA146xx_SDK](https://support.dialog-semiconductor.com/system/files/restricted/SDK_10.0.4.66.2.zip)（示例工程使用SDK版本号SDK_10.0.4.66）
2. 进入到文件目录 SDK_10.0.4.66\projects\dk_apps\demos\
3. 执行 git clone --recursive https://github.com/MiEcosystem/mijia_ble_standard.git -b dialog146xx
4. 联系米家获取认证库 stand-auth-cortex-m33f.a，更名为libstand-auth-cortex-m33.a，放置到SDK_10.0.4.66\sdk\interfaces\ble\binaries\DA1469x-Release目录下
5. 由于SDK和MIJIA 库符号的重名，需要将Dialog SDK所有 queue_init函数修改为dialog_queue_init()
