typedef enum{
    SET,
    RESET
} State;
void main(void) {
    State previousState;
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) {
        previousState = SET;
    } else {
        previousState = RESET;
    }

    while (1)
    {
        State currentState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) ? SET : RESET;
        if (currentState != previousState) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
            previousState = currentState;
        }
        
    }
    
}