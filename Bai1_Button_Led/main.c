
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
	unsigned int CRl;   //32bit = 4byte 0x00
	unsigned int CRH;   //0x04
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
	
    //cap xung 
	RCC->APB2ENR|= (1 << 4) | (1 << 2);	

    //cau hinh pc13
	GPIOC->CRH |= (3 << 20);		
	GPIOC->CRH &= ~(3 << 22);
	

    //cau hinh pa0
    GPIOA->CRl |= 8; //set mode và cnf
		GPIOA->CRl &= ~7;
    GPIOA->ODR |= 1;

	while(1){
        if((GPIOA->IDR & 1) == 0){
            GPIOC->ODR |= (1 << 13);
        }
        else GPIOC->ODR &= ~(1 << 13);
	}
	
}