# H747-Test

This is a mix between using CubeMX to create the project with AzureRTOS enabled on both cores and
adding parts from the `LCD_DSI_CmdMode_DoubleBuffer` example for `STM32H747I_DISCO`

- M4 will toggle the orange LED twice per second using 1 tx thread

- M7 will alternatively display 2 sample images on the LTDC display and blink the blue LED, each
  using 1 tx thread.  The image thread receives an event once the display has received the new image
  to be displayed

## board view

![Board view](board.mp4)

https://user-images.githubusercontent.com/39457172/114636730-0539d300-9cc8-11eb-848a-965ba606d748.mp4
