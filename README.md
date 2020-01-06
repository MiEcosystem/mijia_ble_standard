## 支持的硬件平台

- DA14585
- DA14531

DA1468x/69x硬件请参考 [da146xx 分支](https://github.com/MiEcosystem/mijia_ble_standard/tree/da146xx)

## 使用说明

1. 获取DA145xx_SDK（示例工程使用SDK版本号6.0.12.1020, https://www.dialog-semiconductor.com/products/connectivity/bluetooth-low-energy/products/da14531）
2. 进入到文件目录 6.0.12.1020\projects\target_apps\ble_examples\
3. 执行 git clone --recursive https://github.com/MiEcosystem/mijia_ble_standard.git -b Dialog

4. 注意：由于Dialog平台的特殊性

    a)  需要按照sdk_mijia_change.patch 对SDK做对应修改

    b)  将mijia_ble_libs/common/mible_beacon.c文件中mible_service_data_set函数前面的static标志去掉，保证工程顺利编译通过

    c)  在mijia_ble_libs/cryptography/mi_crypto_backend_uECC.c文件中

```C
    #ifdef __DA14531__
    static const uint32_t g_rng_function __attribute__((at(0x07fc1518))) = NULL;
    #else
    static const uint32_t g_rng_function __attribute__((at(0x07fc2768))) = NULL;
    #endif

    //修改swap_endian函数为
    static int swap_endian(const uint8_t *in, uint8_t *out, uint32_t size)
    {
        if (out < in + size && in < out + size)
            return -1;

    //    for(int i = 0; i < size; i++)
    //        out[i] = in[size-1-i];
        for(int i = 0; i < size; i++)
            out[i] = in[i];
        return 0;
    }
```

5. 可以使用下面脚本命令设置SDK目录和工程目录对应的关系 python.exe dlg_make_keil5_env_v1.004.py --sdkpath SDK_PATH
6. 编译运行DA14585或DA14531工程


更多信息请参考 [**米家标准BLE产品接入指南**](https://github.com/MiEcosystem/miio_open/blob/master/ble/02-标准BLE接入开发.md)
