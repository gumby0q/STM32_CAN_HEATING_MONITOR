

## Can protocol


| id | payload | comment |
| ----------- | ----------- | ----------- |
| 0x0d0 | 1st byte - temperature error `uint8_t`; 2-4 temperature value in `float`  | boiler temperature; error is 0 - Ok; if temperature is -127 - it's substituded value of wrong temperature or error |
| 0x091 | 1st byte - pupms status `uint8_t` |PUMP_STATUS_CAN_ID one bit corresponds to pump status ON or OFF| 
| --- | --- | --- |

