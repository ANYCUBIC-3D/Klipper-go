package project

type ITemperature interface {
	IGetTemperature
	ISetTemperature
}

type IGetTemperature interface {
	Get_temp(eventtime float64) (float64, float64)
}

type ISetTemperature interface {
	Set_temp(degrees float64)
}
