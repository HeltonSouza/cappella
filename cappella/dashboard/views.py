from django.shortcuts import render, get_object_or_404

from .models import VehicleData

# Create your views here.
def index(request):
    # return HttpResponse("Hello, world. You're at the Dashboard index.")
    vehicle = get_object_or_404(VehicleData, pk=1)
    return render(request, "dashboard/index.html", {"vehicle": vehicle})