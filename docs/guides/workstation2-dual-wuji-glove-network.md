# Workstation2 双 Wuji Glove 直连网络配置

状态：`VERIFIED`（2026-07-30）

本文归档 Workstation2 上两只 Wuji Glove 的实际网卡身份、NetworkManager
持久配置、定向路由和排障边界。它只属于主机侧设备接入；不改变项目五层架构、
Isaac Session 或 ROS 2 数据边界。

## 当前拓扑

两只手套各经一根 RJ45 网线和一个 USB-C 千兆以太网扩展坞直连 Workstation2：

```text
左 Glove ── RJ45 ── USB Ethernet (left)  ── Workstation2
右 Glove ── RJ45 ── USB Ethernet (right) ── Workstation2
```

主机常规局域网继续由 `eno1` 使用 `10.0.0.0/24` 和默认路由；两张 Glove
网卡均不允许提供默认路由或 DNS。

| 角色 | Glove 序列号 / endpoint | NetworkManager profile | 网卡 / MAC | 主机地址 | 定向路由 |
|---|---|---|---|---|---|
| 左 | `WG1JA06260702004` / `192.168.1.100:50001` | `wuji-glove-left` | `enx00e04c182c28` / `00:E0:4C:18:2C:28` | `192.168.1.10/24` | `192.168.1.100/32` 走左网卡 |
| 右 | `WG1KA06260623528` / `192.168.1.101:50001` | `wuji-glove-right` | `enx6c1ff7cd0e76` / `6C:1F:F7:CD:0E:76` | `192.168.1.11/24` | `192.168.1.101/32` 走右网卡 |

两张 USB 网卡均为 Realtek RTL8153（`r8152` 驱动）。profile 同时按接口名和
MAC 绑定，避免 USB 枚举顺序变化造成左右角色交换。

## 当前路由模型

两只设备目前保留厂商/既有地址 `192.168.1.100` 与 `192.168.1.101`，因此这不是
两套真正不同的 IP 子网。其稳定性来自两条更具体的 `/32` 主机路由：

```text
192.168.1.100/32 -> enx00e04c182c28, source 192.168.1.10
192.168.1.101/32 -> enx6c1ff7cd0e76, source 192.168.1.11
```

`/32` 比两张网卡各自的 `192.168.1.0/24` connected route 更具体，因而主机能够
把两个明确 endpoint 发往正确的直连链路。运行时应使用显式 endpoint，不应依赖
广播扫描来区分左右设备。

若后续通过 Wuji 官方工具确认可持久修改右手设备 IP，可迁移为真正独立子网：
右手设备改为 `192.168.2.101`、主机右网卡改为 `192.168.2.10/24`，并同步更新
host-local binding。未确认设备侧 IP 写入能力前，不应只修改主机地址或项目 YAML。

## 持久化要求

两个 profile 均应满足：

```text
connection.autoconnect = yes
ipv4.method             = manual
ipv4.never-default      = yes
ipv6.method             = disabled
```

不要把 `ip address replace ...` 作为长期配置：它仅在当前内核接口对象存活期间有效，
USB-C 断开重连后会丢失。应由 NetworkManager 的 `wuji-glove-left` 和
`wuji-glove-right` profile 自动恢复地址与路由。

需要手动激活某一路时：

```bash
sudo nmcli connection up wuji-glove-left ifname enx00e04c182c28
sudo nmcli connection up wuji-glove-right ifname enx6c1ff7cd0e76
```

## 运行前检查

```bash
nmcli -t -f DEVICE,STATE,CONNECTION device status
ip -brief address show enx00e04c182c28
ip -brief address show enx6c1ff7cd0e76
ip route get 192.168.1.100
ip route get 192.168.1.101
ping -I 192.168.1.10 -c 3 192.168.1.100
ping -I 192.168.1.11 -c 3 192.168.1.101
```

验收条件：两张网卡都是 `UP`/`LOWER_UP`，profile 分别为
`wuji-glove-left` 和 `wuji-glove-right`，并且 `.100` 与 `.101` 的路由分别指向
左右网卡。

## 排障边界

`NO-CARRIER` 或 `Link detected: no` 是 RJ45 物理层未建立，不是 IP、SDK 或
Isaac 问题。此时先检查手套采集板供电、RJ45 插接、网线和扩展坞端口；仅重设 IP
不会恢复链路。

2026-07-30 的右侧故障在重新插拔 RJ45 后恢复。恢复后两路均协商为 100 Mbps
全双工，5 次 ping 为 0 丢包，随后 30 秒持续 carrier/ping 检查没有断链。该结果
支持“一次物理插接或自协商未完成”的判断，但不足以证明线材或端口不存在间歇故障。

项目运行时的设备序列号和 endpoint 仍保存在目标机 Git 忽略的
`configs/local/workstation2_nv4_v1.yaml`；本指南只记录主机接入配置，不将其复制到
五层 Session 配置中。
