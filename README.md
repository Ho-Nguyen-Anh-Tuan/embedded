# Embedded 🔠

## Unit 1: Blink LED

Bài học này hướng dẫn cách điều khiển LED bằng cách thao tác trực tiếp trên các thanh ghi của vi điều khiển (MCU). Điều này giúp bạn hiểu rõ cách các ngoại vi hoạt động.

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

### **4. Code mẫu**

Dưới đây là chương trình đầy đủ để điều khiển LED:

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

### **5. Ưu và nhược điểm**

- **Ưu điểm:**
  - Giúp hiểu rõ cách hoạt động của các ngoại vi.
  - Tăng hiệu suất do thao tác trực tiếp trên thanh ghi.

- **Nhược điểm:**
  - Cách thực hiện khá phức tạp và đòi hỏi sự tỉ mỉ.

---

### **6. Xây dựng cấu trúc thanh ghi**

- **Mục đích:** Đơn giản hóa việc thao tác với các thanh ghi bằng cách sử dụng cấu trúc (`struct`).
- **Nguyên tắc:**
  - Địa chỉ của `struct` là địa chỉ của thành viên 

