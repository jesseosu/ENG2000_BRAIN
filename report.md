# Analysis Report for main.c

After a thorough review of `main.c`, several areas for improvement have been identified. The file is extremely large, which poses a significant challenge to maintainability. Breaking it down into smaller, more focused modules would be a good first step. This report categorizes the findings into dead code, inefficiencies, contradictory logic, and code duplication, with actionable recommendations for each.

## 1. Dead Code

### Issue: Unused Test Handlers
The `test_handler` and `toggle_handler` functions appear to be remnants of debugging and are not essential to the application's core functionality. While the `toggle_handler` could be useful for a simple hardware check, it is not integrated into any primary workflow.

**Recommendation:**
Remove these handlers to streamline the codebase. If a hardware test function is needed, it should be part of a more comprehensive diagnostics module.

### Issue: Commented-Out Motor Test
The motor test sequence in `app_main` is commented out. While useful during development, it serves no purpose in the final code and adds to the clutter.

**Recommendation:**
Remove the commented-out motor test code. For future development, such tests should be managed in a separate file or a dedicated test framework.

## 2. Inefficiencies

### Issue: Inefficient Median Filter
The `median_filter` function uses an insertion sort, which has a time complexity of O(n^2). For larger datasets, this can become a performance bottleneck.

**Recommendation:**
Replace the insertion sort with a more efficient algorithm, such as quickselect, which has an average time complexity of O(n). This will improve the performance of the filter, especially if the number of samples is increased.

### Issue: Repetitive File Access
The web server repeatedly opens, reads, and closes static files from SPIFFS for each HTTP request. This is resource-intensive and can be slow.

**Recommendation:**
To optimize file serving, consider embedding the static files directly into the firmware using `idf.py_embed_files`. This approach is much faster and reduces the overhead of SPIFFS access.

### Issue: Memory Fragmentation in SSE Handler
The `sse_handler` creates and destroys a `cJSON` object in a loop, which can lead to memory fragmentation over time.

**Recommendation:**
Refactor the `sse_handler` to reuse the `cJSON` object. Alternatively, build the JSON string manually to avoid the overhead of the `cJSON` library in this performance-critical section.

## 3. Contradictory Logic

### Issue: Incorrect ADC Conversion
In `read_hall_sensor_mv`, the code intended to convert the raw ADC reading to millivolts is commented out, and the raw value is returned instead. This could lead to incorrect sensor readings and improper behavior of the bridge's limit switches.

**Recommendation:**
Uncomment and correct the ADC conversion logic to ensure accurate sensor readings. The formula should be validated against the hardware specifications to guarantee correctness.

### Issue: Ineffective Car Detection
The `bridge_car_on_bridge` function always returns `false`, which means the `BRIDGE_CHECK_CLEAR` state will always pass immediately. While the comments indicate this is a placeholder, it renders the state machine's safety check ineffective.

**Recommendation:**
If car detection is a planned feature, this function should be implemented correctly. If it is not, the related logic in the state machine should be removed to avoid confusion and potential safety issues.

## 4. Code Duplication

### Issue: Redundant Calibration and Detection Logic
There is significant code duplication between the calibration and detection logic for the two boat sensors. Functions like `start_calibration`, `process_calibration_sample`, and `detect_boat_calibrated` are nearly identical for both sensors.

**Recommendation:**
Refactor this logic into a single set of functions that can be used for both sensors. A `struct` can be used to hold the state for each sensor, and a pointer to this `struct` can be passed to the functions.

### Issue: Duplicated HTTP Handlers
The HTTP handlers for serving static files are all very similar. This makes the code harder to maintain and adds unnecessary boilerplate.

**Recommendation:**
Create a single, generic HTTP handler that can serve any static file from SPIFFS based on the request URI. This will reduce code duplication and make the web server easier to manage.

### Issue: Identical Data Buffer Functions
The `add_to_data_buffer` and `add_to_boat2_data_buffer` functions are identical.

**Recommendation:**
Combine these into a single function that takes a pointer to the data buffer and its related state variables as arguments.
