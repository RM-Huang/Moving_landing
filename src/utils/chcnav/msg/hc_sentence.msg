Header header   # header
int16  msg_id   # 协议id，虽然可以从data中解析，但在此处声明更方便后续处理
int8[] data     # hc 协议语句(二进制串) 长度可以使用 data.size() 查看