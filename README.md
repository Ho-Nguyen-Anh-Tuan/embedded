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

<details><summary>Unit</summary>
<p>


</p>
</details>
