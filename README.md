## 支持的硬件平台

- DA14585

DA1468x/69x硬件请参考 [da146xx 分支](https://github.com/MiEcosystem/mijia_ble_standard/tree/da146xx)

## 使用说明

1. 获取DA14585_SDK（示例工程使用SDK版本号6.0.6.427）
2. 进入到文件目录 DA14585_SDK\6.0.6.427\projects\target_apps\ble_examples\
3. 执行 git clone --recursive https://github.com/MiEcosystem/mijia_ble_standard.git -b Dialog
4. 注意：由于Dialog平台的特殊性，需要做如下修改：

    a. 将mijia_ble_libs/common/mible_beacon.c文件中mible_service_data_set函数前面的static标志去掉，保证工程顺利编译通过
    b. 在mijia_ble_libs/cryptography/mi_crypto_backend_uECC.c文件中，
```C
//添加一行代码
static const uint32_t g_rng_function __attribute__((at(0x07fc2768))) = NULL;

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

更多信息请参考 [**米家标准BLE产品接入指南**](https://github.com/MiEcosystem/miio_open/blob/master/ble/02-标准BLE接入开发.md)
