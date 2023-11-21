#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
 
#include <JetsonGPIO.h>
 
using namespace std;
class Elec_mag
{
private:
	void Elec_mag()
	{
		//引脚定义
		int output_pin = 12; // BOARD pin 12, BCM pin 18
 
		//设置BCM模式
		GPIO::setmode(GPIO::BOARD);

		//设置引脚为输出引脚并初始化为高电平
		GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);
	}

public:
	void elecmag_activate(bool seg)
	{
		if(seg)
			GPIO::output(output_pin, 1);
		else
			GPIO::output(output_pin, 0);
	}
}