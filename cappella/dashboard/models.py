from django.db import models

# Create your models here.
class VehicleData(models.Model):
    date = models.DateTimeField("date collected")
    air_tank_1 = models.IntegerField(default=0)
    air_tank_2 = models.IntegerField(default=0)
    rpm = models.IntegerField(default=0)
    speed = models.IntegerField(default=0)

    def __str__(self):
        return str(self.date.strftime("%Y-%m-%d,%H:%M:%S:%f,%Z"))