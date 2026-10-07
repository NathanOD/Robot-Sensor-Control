import os
from cffi import FFI

ffi = FFI()

ffi.cdef("""
    typedef int kStatus;
    
    kStatus gocator_connect(const char* ip_address);
    kStatus gocator_start();
    kStatus gocator_stop();
    kStatus gocator_disconnect();
    kStatus gocator_trigger();
    
    kStatus gocator_receive_data(double* out_buffer, int max_points, int* points_received);
    kStatus gocator_receive_intensity_image(unsigned char* out_buffer, int max_size, 
                                            int* width, int* height, int* size_received);
""")

source_c = """
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <string.h>

static kAssembly api = kNULL;
static GoSystem system_obj = kNULL;
static GoSensor sensor = kNULL;

kStatus gocator_connect(const char* ip_address) {
    kStatus status;
    kIpAddress ipAddress;
    
    if ((status = GoSdk_Construct(&api)) != kOK) { printf("GoSdk_Construct failed: %d\\n", status); fflush(stdout); return status; }
    if ((status = GoSystem_Construct(&system_obj, kNULL)) != kOK) { printf("GoSystem_Construct failed: %d\\n", status); fflush(stdout); return status; }
    
    kIpAddress_Parse(&ipAddress, ip_address);
    if ((status = GoSystem_FindSensorByIpAddress(system_obj, &ipAddress, &sensor)) != kOK) { printf("GoSystem_FindSensorByIpAddress failed: %d\\n", status); fflush(stdout); return status; }
    if ((status = GoSensor_Connect(sensor)) != kOK) { printf("GoSensor_Connect failed: %d\\n", status); fflush(stdout); return status; }
    if ((status = GoSystem_EnableData(system_obj, kTRUE)) != kOK) { printf("GoSystem_EnableData failed: %d\\n", status); fflush(stdout); return status; }
    
    return kOK;
}

kStatus gocator_start() {
    return GoSystem_Start(system_obj);
}

kStatus gocator_stop() {
    return GoSystem_Stop(system_obj);
}

kStatus gocator_receive_data(double* out_buffer, int max_points, int* points_received) {
    GoDataSet dataset = kNULL;
    *points_received = 0;
    kStatus status = GoSystem_ReceiveData(system_obj, &dataset, 20000000);
    
    if (status == kOK) {
        printf("Received dataset with %u items\\n", (unsigned int)GoDataSet_Count(dataset));
        fflush(stdout);
        for (unsigned int i = 0; i < GoDataSet_Count(dataset); ++i) {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);
            printf("Item %u type: %d\\n", i, GoDataMsg_Type(dataObj));
            fflush(stdout);
            
            if (GoDataMsg_Type(dataObj) == 8 /* GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE */) {
                GoSurfaceMsg surfaceMsg = dataObj;
                unsigned int width = GoSurfaceMsg_Width(surfaceMsg);
                unsigned int length = GoSurfaceMsg_Length(surfaceMsg);
                
                double XResolution = ((double)GoSurfaceMsg_XResolution(surfaceMsg)) / 1000000.0;
                double YResolution = ((double)GoSurfaceMsg_YResolution(surfaceMsg)) / 1000000.0;
                double ZResolution = ((double)GoSurfaceMsg_ZResolution(surfaceMsg)) / 1000000.0;
                double XOffset = ((double)GoSurfaceMsg_XOffset(surfaceMsg)) / 1000.0;
                double YOffset = ((double)GoSurfaceMsg_YOffset(surfaceMsg)) / 1000.0;
                double ZOffset = ((double)GoSurfaceMsg_ZOffset(surfaceMsg)) / 1000.0;
                
                for (unsigned int rowIdx = 0; rowIdx < length; rowIdx++) {
                    k16s *data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);
                    for (unsigned int colIdx = 0; colIdx < width; colIdx++) {
                        if (*points_received >= max_points) break;
                        
                        if (data[colIdx] != -32768) { // INVALID_RANGE_16BIT
                            out_buffer[(*points_received)*3]     = XOffset + XResolution * colIdx;      // X
                            out_buffer[(*points_received)*3 + 1] = YOffset + YResolution * rowIdx;      // Y
                            out_buffer[(*points_received)*3 + 2] = ZOffset + ZResolution * data[colIdx];// Z
                            (*points_received)++;
                        }
                    }
                }
            }
        }
        GoDestroy(dataset);
        return kOK;
    } else {
        printf("GoSystem_ReceiveData failed with status: %d\\n", status);
        fflush(stdout);
    }
    return kERROR;
}

kStatus gocator_disconnect() {
    if (system_obj != kNULL) GoDestroy(system_obj);
    if (api != kNULL) GoDestroy(api);
    system_obj = kNULL;
    api = kNULL;
    return kOK;
}

kStatus gocator_trigger() {
    if (sensor != kNULL) {
        return GoSensor_Trigger(sensor);
    }
    return kERROR;
}

kStatus gocator_receive_intensity_image(unsigned char* out_buffer, int max_size,
                                        int* width, int* height, int* size_received) {
    GoDataSet dataset = kNULL;
    *size_received = 0;
    *width = 0;
    *height = 0;
    
    kStatus status = GoSystem_ReceiveData(system_obj, &dataset, 20000000);
    
    if (status == kOK) {
        printf("Received dataset with %u items\\n", (unsigned int)GoDataSet_Count(dataset));
        fflush(stdout);
        
        for (unsigned int i = 0; i < GoDataSet_Count(dataset); ++i) {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);
            printf("Item %u type: %d\\n", i, GoDataMsg_Type(dataObj));
            fflush(stdout);
            
            // Try type 9 (Surface Intensity) or type 11 (Profile Intensity)
            if (GoDataMsg_Type(dataObj) == 9 /* GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY */) {
                GoSurfaceIntensityMsg intensityMsg = dataObj;
                *width = (int)GoSurfaceIntensityMsg_Width(intensityMsg);
                *height = (int)GoSurfaceIntensityMsg_Length(intensityMsg);
                int total_pixels = *width * *height;
                
                printf("Found Surface Intensity: %d x %d\\n", *width, *height);
                fflush(stdout);
                
                if (total_pixels > max_size) {
                    printf("Intensity image too large: %d pixels > %d max\\n", total_pixels, max_size);
                    fflush(stdout);
                    GoDestroy(dataset);
                    return kERROR;
                }
                
                for (int rowIdx = 0; rowIdx < *height; rowIdx++) {
                    unsigned char *row_data = GoSurfaceIntensityMsg_RowAt(intensityMsg, rowIdx);
                    memcpy(&out_buffer[rowIdx * (*width)], row_data, *width);
                }
                *size_received = total_pixels;
                printf("Intensity image captured: %d x %d (%d pixels)\\n", *width, *height, total_pixels);
                fflush(stdout);
                GoDestroy(dataset);
                return kOK;
            }
        }
        
        printf("No surface intensity image found in dataset\\n");
        fflush(stdout);
        GoDestroy(dataset);
    } else {
        printf("GoSystem_ReceiveData failed with status: %d\\n", status);
        fflush(stdout);
    }
    return kERROR;
}
"""

root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
lib_dir = os.path.join(root_dir, 'go-sdk', 'lib', 'linux_x64d')

ffi.set_source("_gocator_wrapper", source_c,
    include_dirs=[
        os.path.join(root_dir, 'go-sdk', 'Gocator', 'GoSdk'),
        os.path.join(root_dir, 'go-sdk', 'Platform', 'kApi')
    ],
    library_dirs=[lib_dir],
    libraries=['GoSdk', 'kApi'],
    extra_link_args=[f'-Wl,-rpath,{lib_dir}']
)

if __name__ == "__main__":
    ffi.compile(verbose=True)
