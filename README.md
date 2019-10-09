## 支持的硬件平台

- DA1468x
- DA1469x

DA14585硬件请参考 [Dialog 分支](https://github.com/MiEcosystem/mijia_ble_standard/tree/Dialog)

## 使用说明

1. 获取[DA146xx_SDK](https://support.dialog-semiconductor.com/system/files/restricted/SDK_10.0.4.66.2.zip)（示例工程使用SDK版本号SDK_10.0.4.66）
2. 进入到文件目录 SDK_10.0.4.66\projects\dk_apps\demos\
3. 执行 git clone --recursive https://github.com/MiEcosystem/mijia_ble_standard.git -b da146xx
4. 联系米家获取认证库 stand-auth-cortex-m33f.a，更名为libstand-auth-cortex-m33.a，放置到SDK_10.0.4.66\sdk\interfaces\ble\binaries\DA1469x-Release目录下
5. 由于SDK和MIJIA 库符号的重名，需要将Dialog SDK所有 queue_init函数修改为dialog_queue_init()
6. 注意：由于Dialog146xx平台的特殊性，需要做如下修改：

    a. 将mijia_ble_libs/common/mible_beacon.c文件中mible_service_data_set函数前面的static标志去掉，保证工程顺利编译通过  
    b. 在mijia_ble_libs\common\mible_crypto.c文件中，修改mi_session_uninit函数  
```C
int mi_session_uninit(void)
{
    if (m_flags.initialized == 1) {
        m_flags.initialized = 0;
        mible_timer_stop(session_timer);
        mible_timer_delete(session_timer);
        session_timer = NULL;
    }
    return 0;
}
```  


    c. 在mijia_ble_libs\mijia_profiles\stdio_service_server.c文件中，修改gatts_event_handler函数  
```C
static void gatts_event_handler(mible_gatts_evt_t evt,
    mible_gatts_evt_param_t* param)
{
    switch (evt) {
    case MIBLE_GATTS_EVT_WRITE:
        on_write_permit(param);
        break;
    case MIBLE_GATTS_EVT_READ_PERMIT_REQ:
        break;
    case MIBLE_GATTS_EVT_WRITE_PERMIT_REQ:
        on_write_permit(param);
        break;
    case MIBLE_GATTS_EVT_IND_CONFIRM:
        break;
    }
}
```

更多信息请参考 [**米家标准BLE产品接入指南**](https://github.com/MiEcosystem/miio_open/blob/master/ble/02-标准BLE接入开发.md)
