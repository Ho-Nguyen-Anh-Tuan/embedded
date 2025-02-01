# Embedded 💻
<details><summary>Unit 1: Set up Keil and blink led</summary>
<p>

## Unit 1: Set up Keil and blink led

Điều khiển LED bằng cách thao tác trực tiếp trên các thanh ghi của vi điều khiển (MCU).

---

### **1. Cấp clock cho ngoại vi**

- **Mục đích:** Kích hoạt xung clock cho chân GPIOC.
- **Cách thực hiện:**
  - Địa chỉ bắt đầu của RCC: `0x40021000`.
  - Độ dời địa chỉ APB2: `0x18`.
  - Địa chỉ của thanh ghi RCC_APB2ENR: `0x40021018`.

- **Thao tác:**
  - Bật xung clock cho GPIOC tại **bit 4** bằng kỹ thuật bitmask:
    ```c
    RCC_APB2ENR |= (1 << 4);
    ```

---

### **2. Cấu hình chế độ chân GPIO**

- **Mục đích:** Thiết lập chân PC13 làm ngõ ra (output) với tốc độ tối đa 50MHz.
- **Cách thực hiện:**
  - Địa chỉ PORT C: `0x40011000`.
  - Độ dời của thanh ghi CRH: `0x04`.
  - Địa chỉ thanh ghi GPIOC_CRH: `0x40011004`.

- **Thao tác:**
  - Thiết lập **MODE13 = 11** để chọn output mode với tốc độ tối đa 50MHz:
    ```c
    GPIOC_CRH |= (3 << 20);
    ```
  - Thiết lập **CNF13 = 00** để chọn chế độ output push-pull:
    ```c
    GPIOC_CRH &= ~(3 << 22);
    ```

---

### **3. Sử dụng ngoại vi**

- **Mục đích:** Điều khiển LED bật/tắt thông qua chân PC13.
- **Cách thực hiện:**
  - Địa chỉ thanh ghi ODR: `0x4001100C`.

- **Thao tác:**
  - Set **bit 13** của thanh ghi ODR để bật LED:
    ```c
    GPIOC_ODR |= (1 << 13);
    ```
  - Clear **bit 13** để tắt LED:
    ```c
    GPIOC_ODR &= ~(1 << 13);
    ```

---

#### **Code mẫu**


```c
#define RCC_APB2ENR *((unsigned int *)0x40021018)
#define GPIOC_CRH    *((unsigned int *)0x40011004)
#define GPIOC_ODR    *((unsigned int *)0x4001100C)

void delay(unsigned int timeDelay){
    for(unsigned int i = 0; i < timeDelay; i++){}
}

int main(){
    RCC_APB2ENR |= (1 << 4);    // Cấp xung clock cho GPIOC
    GPIOC_CRH |= (3 << 20);     // Mode13 = 11, output mode, max speed 50MHz
    GPIOC_CRH &= ~(3 << 22);    // CNF13 = 00, output push-pull
    
    while(1){
        GPIOC_ODR |= (1 << 13);  // Bật LED
        delay(1000000);          // Delay
        GPIOC_ODR &= ~(1 << 13); // Tắt LED
        delay(1000000);          // Delay
    }
}
```

---

#### **Ưu và nhược điểm**

- **Ưu điểm:**
  - Giúp hiểu rõ cách hoạt động của các ngoại vi.
  - Tăng hiệu suất do thao tác trực tiếp trên thanh ghi.

- **Nhược điểm:**
  - Cách thực hiện khá phức tạp.

---

### **3.5 Xây dựng cấu trúc thanh ghi**

- **Mục đích:** Đơn giản hóa việc thao tác với các thanh ghi bằng cách sử dụng cấu trúc (`struct`).
- **Nguyên tắc:**
  - Địa chỉ của `struct` là địa chỉ của thành viên đầu tiên, các thành viên tiếp theo ứng với cấu trúc thực tế của MCU.
  #### Code:

  
```c
  

typedef struct{
	unsigned int CR;
	unsigned int CFGR;
	unsigned int CIR;
	unsigned int APB2RSTR;
	unsigned int CAB1RSTR;
	unsigned int AHBENR;
	unsigned int APB2ENR;
	unsigned int APB1ENR;
	unsigned int BDCR;
	unsigned int CSR;
} RCC_typeDef;

typedef struct{
	unsigned int CRl;	//32bit = 4byte 0x00
	unsigned int CRH; //0x04
	unsigned int IDR;	//0x08
	unsigned int ODR;
	unsigned int BSRR;
	unsigned int BRR;
	unsigned int LCKR;
} GPIO_typeDef;

#define RCC		((RCC_typeDef *)0x40021000)
#define GPIOC ((GPIO_typeDef *)0x40011000)
#define GPIOA ((GPIO_typeDef *)0x40010800)

void delay(unsigned int timeDelay){
	for(unsigned int i = 0; i < timeDelay; i++){}
}

int main(){
	
	RCC->APB2ENR|= (1 << 4);	
	GPIOC->CRH |= (3 << 20);		
	GPIOC->CRH &= ~(3 << 22);
	
	while(1){
		GPIOC->ODR |= (1 << 13);
		delay(1000000);
		GPIOC->ODR &= ~(1 << 13);
		delay(1000000);
	}
	
}
```  

### Điều khiển LED PC13 qua nút nhấn nối ở PA0 

#### **1. Bật xung clock cho ngoại vi**

- Cấp xung clock cho GPIOA và GPIOC thông qua thanh ghi APB2 bằng kỹ thuật bitmask.
- Bật bit 4 và bit 2 của thanh ghi APB2.

#### **2. Cấu hình chế độ chân**

- Đối với PA0:
  - Set MODE = `00` (input mode).
  - Set CNF = `10` để chọn chế độ input pull-up/pull-down.
  - Đặt ODR = `1` (input pull-up). Nếu ODR = `0`, chế độ sẽ là input pull-down.
- Đối với PC13:
  - Set MODE = `11` (output mode, max speed 50MHz).
  - Set CNF = `00` (output push-pull).

#### **3. Sử dụng ngoại vi**

- Đọc mức điện áp từ thanh ghi `IDR` của GPIOA bằng phép AND để kiểm tra trạng thái của nút nhấn.
- Dựa trên trạng thái đọc được:
  - Nếu PA0 ở mức thấp, bật LED PC13.
  - Nếu PA0 ở mức cao, tắt LED PC13.

#### **Code ví dụ**

```c
typedef struct {
    unsigned int CR;
    unsigned int CFGR;
    unsigned int CIR;
    unsigned int APB2RSTR;
    unsigned int APB1RSTR;
    unsigned int AHBENR;
    unsigned int APB2ENR;
    unsigned int APB1ENR;
    unsigned int BDCR;
    unsigned int CSR;
} RCC_typeDef;

typedef struct {
    unsigned int CRL;   // 0x00
    unsigned int CRH;   // 0x04
    unsigned int IDR;   // 0x08
    unsigned int ODR;   // 0x0C
    unsigned int BSRR;  // 0x10
    unsigned int BRR;   // 0x14
    unsigned int LCKR;  // 0x18
} GPIO_typeDef;


#define RCC     ((RCC_typeDef *)0x40021000)
#define GPIOC   ((GPIO_typeDef *)0x40011000)
#define GPIOA   ((GPIO_typeDef *)0x40010800)


void delay(unsigned int timeDelay) {
    for (unsigned int i = 0; i < timeDelay; i++) {}
}

int main() {
    // Bật xung clock cho GPIOA và GPIOC
    RCC->APB2ENR |= (1 << 4) | (1 << 2);

    // Cấu hình PC13 làm output
    GPIOC->CRH |= (3 << 20);       
    GPIOC->CRH &= ~(3 << 22);      

    // Cấu hình PA0 làm input pull-up
    GPIOA->CRL |= (8);             // MODE0 = 00, CNF0 = 10 (input pull-up/pull-down)
    GPIOA->ODR |= 1;               // Set ODR0 = 1 (pull-up)

    while (1) {
        if ((GPIOA->IDR & 1) == 0) { 
            GPIOC->ODR |= (1 << 13); 
        } else {
            GPIOC->ODR &= ~(1 << 13); 
        }
    }
}
```

</p>
</details>

<details><summary>Unit 2: Cấu hình GPIO sử dụng thư viện SPL</summary>
<p>
	
## Unit 2: Cấu hình GPIO sử dụng thư viện SPL

### 1. Blink LED PC13

#### 1. Cấp clock cho ngoại vi
GPIOC nối với bus APB2, do đó sử dụng hàm `RCC_APB2PeriphClockCmd` để cấp clock.
- Hàm nhận 2 tham số:
  - Ngoại vi muốn cấp clock.
  - Cho phép (ENABLE) hoặc không cho phép (DISABLE).

#### 2. Cấu hình ngoại vi
Thư viện SPL cung cấp struct `GPIO_InitTypeDef` với các thành viên:
- `GPIO_Pin`: Chân GPIO muốn cấu hình.
- `GPIO_Speed`: Tốc độ cho bộ GPIO hoạt động.
- `GPIO_Mode`: Chế độ hoạt động.

Chọn `GPIO_Pin_13`, output push-pull, max speed 50MHz. Sử dụng hàm `GPIO_Init` lưu cài đặt vào thanh ghi.

#### 3. Sử dụng ngoại vi
Dùng hàm `GPIO_SetBits` và `GPIO_ResetBits` để bật tắt LED, kết hợp với delay để nháy LED.

### Code:
```c
#include "stm32f10x.h"      // Device header
#include "stm32f10x_rcc.h"  // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h" // Device:StdPeriph Drivers:GPIO

// Cấp xung cho GPIOC
void RCC_Config() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

// Cấu hình chân
void GPIO_Config() {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void delay(unsigned int time) {
    for (int i = 0; i < time; i++) {}
}

int main() {
    RCC_Config();
    GPIO_Config();

    while (1) {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        delay(1000000);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        delay(1000000);
    }
}
```

---

### 2. Chase LED

#### 1. Cấp clock cho ngoại vi
GPIOC chung bus APB2, việc cấp clock tương tự.

#### 2. Cấu hình ngoại vi
Sử dụng `GPIO_Pin_4`, `GPIO_Pin_5`, `GPIO_Pin_6`, `GPIO_Pin_7` cùng chế độ output push-pull, speed 50MHz. Vì bản chất các chân Pin là mask, nên khi ta muốn làm việc với nhiều chân
chỉ cần | (OR) bọn nó với nhau.  
Cài đặt xong lưu cài đặt bằng hàm `GPIO_Init`.

#### 3. Sử dụng ngoại vi
Viết hàm `chaseLed` bằng vòng lặp `for` + `shift bit` để tạo hiệu ứng nháy LED. Ghi giá trị vào thanh ghi `ODR` bằng hàm `GPIO_Write`.

### Code:
```c
#include "stm32f10x.h"      // Device header
#include "stm32f10x_rcc.h"  // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h" // Device:StdPeriph Drivers:GPIO

// Cấp xung cho GPIOC
void RCC_Config() {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
}

// Cấu hình chân
void GPIO_Config() {
    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void delay(uint32_t time) {
    for (uint32_t i = 0; i < time; i++) {}
}

void chaseLed(uint8_t loop) {
    uint16_t Ledval;
    for (int i = 0; i < loop; i++) {
        Ledval = 0x0010; // 0b0001 0000
        for (uint8_t j = 0; j < 4; j++) {
            GPIO_Write(GPIOC, Ledval);
            Ledval <<= 1;
            delay(10000000);
        }
    }
}

int main() {
    RCC_Config();
    GPIO_Config();

    while (1) {
        chaseLed(4);
        break;
    }
}
```

---

### 3. Đọc nút nhấn PA0 - Xuất tín hiệu PC13

#### 1. Cấp clock
GPIOC và GPIOA chung đường bus APB2, khi dùng hàm cấp clock, | (OR) thêm GPIOA.

#### 2. Cấu hình ngoại vi
- Cài đặt Pin = GPIO_Pin_0, Mode input pull-up.
- Lưu cài đặt vào GPIOA bằng hàm `GPIO_Init`.

#### 3. Sử dụng ngoại vi
Trong vòng lặp `while`, kiểm tra trạng thái nút nhấn bằng hàm `GPIO_ReadInputDataBit`. Khi nhấn, chờ nhả nút thả ra rồi thao tác với PC13.
- Đọc giá trị từ PC13, nếu = 1 thì `ResetBits`, ngược lại thì `Setbits`.

Code:  
```c
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_rcc.h"              // Device:StdPeriph Drivers:RCC
#include "stm32f10x_gpio.h"             // Device:StdPeriph Drivers:GPIO

// cap xung cho GPIOC
void RCC_Config(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
}

//cau hinh chan
void GPIO_Config(){
	GPIO_InitTypeDef GPIO_InitStruct;
	
	// cat dai pc13
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;		
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	// cai dat pa0
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}	

int main(){
	RCC_Config();
	GPIO_Config();
	
	while(1){
		if(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){
			while(!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0));
			if(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13)){
				GPIO_ResetBits(GPIOC, GPIO_Pin_13);
			} else{
				GPIO_SetBits(GPIOC, GPIO_Pin_13);
			}
		}
	}
}

```	

</p>
</details>

<details><summary>Unit 3: Interupt - Timer</summary>
<p>

## Unit 3: Interrupt - Timer

### 1.1 Định nghĩa ngắt
Ngắt là một sự kiện khẩn cấp xảy ra bên trong hoặc bên ngoài MCU. Khi xảy ra, MCU sẽ tạm dừng chương trình chính để thực thi chương trình ngắt (trình phục vụ ngắt - ISR).

### 1.2 Các ngắt thông dụng
- Mỗi ngắt có một trình phục vụ ngắt riêng (ISR), tức là mỗi loại sự kiện ngắt sẽ có một ISR cụ thể.
- Trình phục vụ ngắt (Interrupt Service Routine - ISR) là một đoạn chương trình được thực hiện khi ngắt xảy ra.
- Mỗi ISR sẽ có một địa chỉ bắt đầu trong bộ nhớ, được gọi là **vector ngắt**.

  ![Bai3](https://github.com/user-attachments/assets/157b6bc2-9b9e-4097-9b00-587e2a5d6e4a)


- Mỗi ngắt có 1 vector ngắt (ISR)  
- Cờ ngắt là tác nhân gây ra sự kiện ngắt
###  VD: 
Khi IE0 = 1, mcu sẽ biết có 1 ngắt ngoài xảy ra thì nó sẽ chạy đến ISR ngắt ngoài để thực thi.  
**PC (Program ccounter)** là thanh ghi luôn chỉ đến lệnh tiếp theo của chương trình.  
![Bai3 1](https://github.com/user-attachments/assets/2431301c-3ee8-4154-8d1f-fb2a83579b9d)


#### Hoạt động của PC và MSP:
- Khi chương trình chính đang thực thi, nếu xảy ra ngắt, PC (Program Counter) sẽ lưu lại địa chỉ lệnh tiếp theo và nhảy đến địa chỉ của ISR.
- Sau khi ISR kết thúc, PC sẽ lấy lại địa chỉ từ MSP (Main Stack Pointer) và tiếp tục thực thi chương trình chính.

### 1.2.1 Ngắt ngoài
Xảy ra khi có sự thay đổi điện áp ở chân GPIO được cấu hình làm ngõ vào ngắt. Có bốn dạng:
- **Low:** Kích hoạt ngắt liên tục khi chân ở mức 0.
- **High:** Kích hoạt ngắt liên tục khi chân ở mức 1.
- **Rising:** Kích hoạt khi có xung cạnh lên.
- **Falling:** Kích hoạt khi có xung cạnh xuống.

### 1.2.2 Ngắt Timer
Xảy ra khi giá trị trong thanh ghi đếm của Timer bị tràn. Sau mỗi lần tràn, cần reset giá trị thanh ghi để tạo ngắt tiếp theo.
- **Up counter:** Đếm lên.
- **Down counter:** Đếm xuống.

### 1.2.3 Ngắt truyền thông
Xảy ra khi có sự kiện truyền/nhận dữ liệu giữa MCU và các thiết bị khác, thường dùng cho các giao thức như UART, SPI, I2C để đảm bảo dữ liệu chính xác.

### 1.3 Độ ưu tiên ngắt
- Ngắt có độ ưu tiên cao hơn sẽ được thực thi trước, ngắt có độ ưu tiên thấp hơn sẽ chờ.
- Trên STM32, ngắt có số ưu tiên càng thấp thì quyền càng cao.
- Stack Pointer lưu địa chỉ của chương trình chính hoặc ISR đang thực thi dở khi xảy ra ngắt.

### 2. Timer
Timer là một mạch digital logic có vai trò đếm các chu kỳ xung clock, có thể đếm lên hoặc đếm xuống.

#### Ứng dụng của Timer:
1. **Đếm sự kiện:** Mỗi sự kiện là một chu kỳ xung clock.
2. **Delay:**
   - Sử dụng struct `TIM_TimeBaseInitTypeDef` để cấu hình:
     - `TIM_ClockDivision`: Chia tần số từ hệ thống thành các xung clock có tần số nhỏ hơn.
     - `TIM_Prescaler`: bao nhiêu chu kì xung clock mới đếm lên 1 lần, từ đây ta quyết định 1 lần đếm tốn bao nhiêu s.
     - `TIM_Period`: Bao nhiêu lần đếm thì timer tràn.
     - `TIM_CounterMode`: Chế độ đếm (lên hoặc xuống).
   - Gọi hàm `TIM_TimeBaseInit` để lưu cài đặt vào thanh ghi.
   - Hàm `TIM_Cmd` dùng để bật/tắt Timer.
   - Sử dụng hàm `SetCounter` để đặt giá trị đếm ban đầu và `GetCounter` để lấy giá trị đếm hiện tại.

#### Các chế độ đếm:
- **Up:** Từ 0 đến giá trị `Period`.
- **Down:** Từ giá trị `Period` về 0.

#### Code
```c
// Cau hinh Timer
void TIM_Config(){
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	
	// Goal: count up 1 every 0.1ms
	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1; // xung clock mcu : 1 = 72MHz
	TIM_InitStruct.TIM_Prescaler = 7200 - 1;	// 0,1ms count++
	TIM_InitStruct.TIM_Period = 0xFFFF;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);
	TIM_Cmd(TIM2, ENABLE);
}	

void delay_ms(uint32_t TimeDelay){
	TIM_SetCounter(TIM2, 0);
	while(TIM_GetCounter(TIM2) < TimeDelay * 10);
}
```



</p>
</details>

<details><summary>Unit 4: Communication Protocal</summary>
<p>

## Unit 4: Communication Protocol

### 1. Truyền nhận dữ liệu
Là quá trình trao đổi tín hiệu điện áp giữa các chân MCU. Khi MCU A truyền dữ liệu cho MCU B, dữ liệu này sẽ được chuyển đổi thành tín hiệu điện áp trên các chân tương ứng.

- **Vấn đề**: Các bit giống nhau liền kề -> các chuẩn giao tiếp.
- **Chuẩn giao tiếp**: Thống nhất thời điểm 2 MCU bên đọc - bên nhận.

#### Các kiểu giao tiếp:
- **Đơn công**: 1 thiết bị chỉ thực hiện *1 nhiệm vụ duy nhất* (truyền hoặc nhận).
- **Bán song công**: Tại 1 thời điểm, thiết bị chỉ thực hiện *truyền hoặc nhận*.
- **Song công**: Vừa _nhận vừa truyền cùng lúc_.

#### Chuẩn giao tiếp:
- **Song song**: 8 đường dây - mỗi dây truyền 1 bit -> truyền 8 bit 1 lần. Dùng khi yêu cầu về tốc độ.
- **Nối tiếp**: 1 đường dây để truyền tín hiệu, các bit lần lượt truyền trên 1 đường dây.
- **Đồng bộ**: Thống nhất thời điểm 1 thiết bị truyền - 1 thiết bị nhận. Giữa 2 thiết bị có 1 đường dây trực tiếp để đồng bộ.

---

### 2. SPI (Serial Peripheral Interface)
- Chuẩn giao tiếp nối tiếp.
- Đồng bộ.
- 1 master có thể kết nối với nhiều slave.

#### Sử dụng 4 dây:
1. **SCK (Serial Clock)**: Master cung cấp xung đồng bộ.
2. **MISO (Master Input Slave Output)**: ***Master nhận*** dữ liệu từ slave.
3. **MOSI (Master Output Slave Input)**: ***Master truyền*** dữ liệu cho slave.
4. **SS/CS (Slave Select/Chip Select)**: Master chọn slave giao tiếp bằng cách kéo SS xuống mức `0`.  

![slide6](https://github.com/user-attachments/assets/4dfa506a-5a6c-49d3-92cc-6d5275594957)


#### Quá trình hoạt động:
- Master kéo `CS = 0` để chọn slave.
- CLock được cấp bởi master, với mỗi xung clock, 1 bit sẽ được master truyền cho slave và ngược lại.
- Các thanh ghi cập nhật giá trị và dịch 1 bit.
- Lập lại quá trình trên đến khi đủ 8 bit dữ liệu. Sau khi xong thì kéo chân `CS = 1`.  
_Tại 1 thời điểm xung clock, SPI có thể cùng lúc truyền-nhận thông qua MOSI-MISO_.

#### Chế độ hoạt động:
1. **CPOL (Clock Polarity):**
   - `0`: thời điểm **không truyền** dữ liệu (Idle) thì **SCK = 0** -> khi truyền SCK = 1.
   - `1`: Thời điểm **không truyền** dữ liệu, **SCK = 1** -> khi truyền SCK = 0.
2. **CPHA (Clock Phase):**
   - `0`: **nhận** dữ liệu ở **cạnh thứ 1** của xung clock, **truyền** dữ liệu ở **cạnh thứ 2**.
   - `1`: **nhận** dữ liệu ở **cạnh thứ 2** của xung clock, **truyền** dữ liệu ở **cạnh thứ 1**.

![slide9](https://github.com/user-attachments/assets/21fa8007-e8cd-40c6-b21a-3af9b024f36f)

---

### 3. I2C (Inter-Integrated Circuit)
- Chuẩn giao tiếp nối tiếp.
- Đồng bộ.  
- Chế độ bán song công.
- 1 master kết nối với nhiều slave.

#### Sử dụng 2 dây:
1. **SCL (Serial Clock):** Tạo xung đồng bộ.
2. **SDA (Serial Data):** Chân truyền dữ liệu.

#### I2C hoạt động khá đặc biệt: Open drain
Khi mà nó muốn điều khiển thì kéo đường dây `= 0`, khi không điều khiển thì thả trôi đường dây. Nên phải cần 2 con điện trở kéo lên nguồn để `= 1`.

#### Quá trình hoạt động:
1. Không truyền dữ liệu: Cả `SCL và SDA = 1`.
2. Khi truyền cần 1 điều kiện bắt đầu (**start condition**): đưa `SDA = 0` *trước SCL*.
3. Mỗi slave có 1 address riêng, **master gửi 8 bit đầu cho tất cả các slave**   (Call). 7 bit đầu là địa chỉ của slave mà master muốn giao tiếp, bit 8 để xác định đọc hay gửi dữ liệu cho slave (`gửi = 0`, `đọc = 1`).   
4. **Bit ACK**: Bên nhận khi nhận thành công thì kéo `SDA = 0`. Trong 1 chu kì xung clock, bên gửi đọc SDA, nếu `= 0` thì truyền thành công, không thì truyền lại.  
5. Sau khi xong 8 bit đầu thì truyền 8 bit data, sau mỗi 8 bit cũng là ACK.  
6. Truyền xong dữ liệu thì tạo **stop condition**: đưa `SDA = 1` sau SCL .

---

### 4. UART (Universal Asynchronous Receiver-Transmitter)
- Giao tiếp nối tiếp.
- Không đồng bộ.
- Chỉ 2 thiết bị giao tiếp bình đẳng.  
- Chế độ song công.

#### Sử dụng 2 dây:
1. **Tx (Transmit):** Chân truyền dữ liệu.
2. **Rx (Receive):** Chân nhận dữ liệu. 

#### baudrate = số bit truyền được/1s
##### VD: baudrate = 9600  

| 9600 bits | 1000 ms|
| ----------| --------- | 
| 1 bit     | ? ms | 

=> 0.10417 ms  
=> Timer(0 -> 0.10417)
  2 thiết bị quyết định thời điểm gửi bằng timer

#### Quá trình hoạt động: Tuy UART không có đường dây đồng bộ nhưng cũng dựa theo cơ chế _truyền - ngắt_ tương tự như SPI hay I2C. 
1. Bên nào muốn gửi thì tạo 1 **start bit** (đưa Tx: `1->0`), sau đó delay khoảng thời gian 1 bit (0.10417 ms).

2. `if (Rx == 1)` dùng để liên tục kiểm tra Tx của MCU A, khi MCU A thực hiện delay thì MCU B cũng thực delay với cùng 1 khoảng thời gian. Lúc này tín hiệu `Tx = 0` của MCU A đã ổn định, MCU B đọc Rx của nó thì thấy đã có bit start thì bắt đầu truyền dữ liệu.  
![slide19](https://github.com/user-attachments/assets/e55ea27c-8c5a-447c-a1fd-e681a638af7d) 
3. Ở MCU A, gán bit thấp nhất (LSB) cho Tx A, sau đó dịch data qua phải để sẵn sàng truyền bit kế. Lặp lại truyền và delay cho đến khi đủ 8 bit data.

4. MCU B khi nhận bit start thì delay 0.1ms để chờ data. Đọc từ Rx B, nếu `Rx = 1` thì `arr |= mask byte`, xong thì dịch mask byte qua trái và delay chờ bit tiếp theo.
![slide20](https://github.com/user-attachments/assets/f952a088-9c12-46b4-a261-e1ddc9f7bbfb)
5. **Parity** dùng để kiểm tra lỗi trong UART, theo cơ chế đếm số bit 1 trong dữ liệu truyền đi (data + parity).
#### Có 2 loại
 ##### Even parity (chẵn):
 - Số bit `1` trong dữ liệu (bao gồm cả parity bit) luôn là **chẵn**.
 - Nếu số bit 1 trong dữ liệu là **lẻ**, parity bit sẽ được set thành `1` để **tổng số bit 1 là _chẵn_**.
 - Nếu số bit 1 trong dữ liệu **đã chẵn**, parity bit sẽ là `0`.  
#### VD
data | parity bit | truyền đi (data + parity)|
|----|------------|----------|
1010 |  0	  |  10100 |
1101 |  1	  |  11011 |

 ##### Odd parity (lẻ):
  - Số bit `1` trong dữ liệu (bao gồm cả parity bit) luôn là **lẻ**.
  - Nếu số bit 1 trong dữ liệu là **chẵn**, parity bit sẽ được set thành `1` để **tổng số bit 1 là _lẻ_**.
  - Nếu số bit 1 trong dữ liệu **đã lẻ**, parity bit sẽ là `0`.
#### VD
data | parity bit | truyền đi (data + parity)|
|----|------------|----------|
1010 |  1	  |  10101 |
1101 |  0	  |  11010 |

Bên gửi sẽ thêm parity tùy theo cài đặt vào data và truyền đi. Bên nhận sẽ nhận data + parity bit, sau đó kiểm tra số lượng bit `1` **trong toàn bộ dữ liệu _(bao gồm parity)_** khớp với parity đã đặt (even/odd) hay không, nếu không thì báo lỗi.

***Nhược điểm***: Không phát hiện được lỗi nhiều bit.

5. **Stop bit** tùy cấu hình 1-2 stop bits, MCU A đưa Tx: `0->1` xong delay thời gian 1 bit. MCU B đọc Rx, delay chờ Rx ổn định rồi khi thấy `Rx = 1` thì tiến hành stop.
![slide22](https://github.com/user-attachments/assets/6c5dc65a-0b05-4226-ae9c-a11c39eaf40d)

---


</p>
</details>

<details><summary>Unit</summary>
<p>


</p>
</details>

<details><summary>Unit</summary>
<p>


</p>
</details>

<details><summary>Unit</summary>
<p>


</p>
</details>

<details><summary>Unit</summary>
<p>


</p>
</details>
