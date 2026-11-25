// Extension for the existing PaperCrop 2.3 native CLI.
// The original implementation is preserved inside papercrop_cli_legacy.cpp.
#define main papercrop_legacy_main
#include "papercrop_cli_legacy.cpp"
#undef main
#include "robust_detector.hpp"

#include <iomanip>
#include <numeric>
#include <set>
#include <sstream>

// A3 and A4 share the same aspect ratio. The auto mode deliberately only
// recognizes typical landscape A3 >= about 300 dpi. For other scan DPI values
// use --a3-mode force when the scanner is known to have produced landscape A3.
// Never infer A3 solely from aspect ratio.
static bool isLandscapeA3(const cv::Mat& source, const std::string& mode, const fs::path& inputPath) {
    if (mode == "off") return false;
    if (mode != "force" && mode != "auto")
        throw std::runtime_error("Invalid --a3-mode; expected auto, force or off");
    if (mode == "force") {
        if (source.cols <= source.rows)
            throw std::runtime_error("A3 split requires a LANDSCAPE scan (left/right A4). Rotate the input first");
        return true;
    }
    const robust::Dpi dpi = robust::readJfifDpi(inputPath);
    if (dpi.x >= 60 && dpi.y >= 60 && dpi.x <= 1200 && dpi.y <= 1200) {
        // JPEG's JFIF DPI identifies actual paper size. For these scanner
        // files 3296x2331 at 200 DPI means 419x296 mm = landscape A3,
        // whereas 1646x2331 at 200 DPI means 209x296 mm = portrait A4.
        const double physicalW = source.cols * 25.4 / dpi.x;
        const double physicalH = source.rows * 25.4 / dpi.y;
        return source.cols > source.rows &&
               physicalW >= 382 && physicalW <= 464 &&
               physicalH >= 270 && physicalH <= 325;
    }
    double ratio = double(source.cols) / std::max(1, source.rows);
    return source.cols > source.rows && ratio > 1.33 && ratio < 1.50 &&
           source.cols >= 4150 && source.rows >= 2900;
}

static int borderMedian(const cv::Mat& gray) {
    std::vector<unsigned char> pixels;
    const int step = std::max(1, std::min(gray.cols, gray.rows) / 500);
    int thickness = std::max(2, std::min(gray.cols, gray.rows) / 100);
    for (int x = 0; x < gray.cols; x += step) {
        pixels.push_back(gray.at<unsigned char>(thickness / 2, x));
        pixels.push_back(gray.at<unsigned char>(gray.rows - 1 - thickness / 2, x));
    }
    for (int y = 0; y < gray.rows; y += step) {
        pixels.push_back(gray.at<unsigned char>(y, thickness / 2));
        pixels.push_back(gray.at<unsigned char>(y, gray.cols - 1 - thickness / 2));
    }
    std::nth_element(pixels.begin(), pixels.begin() + pixels.size() / 2, pixels.end());
    return pixels[pixels.size() / 2];
}

static double intersectionOverSmaller(const cv::Rect& a, const cv::Rect& b) {
    const double overlap = double((a & b).area());
    const double smaller = double(std::min(a.area(), b.area()));
    return smaller <= 0 ? 0 : overlap / smaller;
}

static std::vector<cv::Rect> rightSideDocuments(const cv::Mat& right) {
    // Production scans have black scanner beds with tiny photos anywhere
    // on the right-hand A4. Use a robust dominant-interior-background
    // detector instead of trusting bright/noisy scanner frame borders.
    if (robust::dominantLightness(right) <= 48) {
        auto boxes = robust::darkRects(right);
        std::sort(boxes.begin(), boxes.end(), [](const auto& a, const auto& b) {
            if (std::abs(a.y - b.y) > std::max(15, std::min(a.height, b.height) / 4))
                return a.y < b.y;
            return a.x < b.x;
        });
        return boxes;
    }
    double scale = std::min(1.0, 1900.0 / std::max(right.cols, right.rows));
    cv::Mat preview;
    if (scale < 1.0)
        cv::resize(right, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    else
        preview = right;

    cv::Mat gray, blurred, edges, contrast, mask;
    cv::cvtColor(preview, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);
    const int bg = borderMedian(blurred);
    cv::absdiff(blurred, cv::Scalar(bg), contrast);
    cv::threshold(contrast, mask, 22, 255, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    cv::Canny(blurred, edges, 35, 105);
    cv::dilate(edges, edges, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7)));

    std::vector<cv::Rect> proposals;
    auto collect = [&](const cv::Mat& binary, int retrieval) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary.clone(), contours, retrieval, cv::CHAIN_APPROX_SIMPLE);
        const double imageArea = double(preview.rows) * preview.cols;
        for (const auto& c : contours) {
            if (c.size() < 4) continue;
            cv::Rect b = cv::boundingRect(c);
            if (b.width < 45 || b.height < 45) continue;
            const double fraction = double(b.area()) / imageArea;
            if (fraction < 0.0007 || fraction > 0.80) continue;
            if (double(b.width) / b.height < 0.25 || double(b.width) / b.height > 4.0)
                continue;
            // Ignore long scanner artefacts and the edge of the entire A4 half.
            if (b.width >= preview.cols * 0.96 || b.height >= preview.rows * 0.96)
                continue;
            std::vector<cv::Point> hull;
            cv::convexHull(c, hull);
            const double envelope = cv::contourArea(hull);
            if (envelope < double(b.area()) * 0.55) continue;
            proposals.push_back(b);
        }
    };
    collect(mask, cv::RETR_EXTERNAL);
    collect(edges, cv::RETR_LIST);
    // Keep every separate qualifying document; largest outline wins over nested
    // internal picture features and duplicate borders. No fixed output count.
    std::sort(proposals.begin(), proposals.end(), [](const auto& a, const auto& b) {
        return a.area() > b.area();
    });
    std::vector<cv::Rect> chosen;
    for (const auto& b : proposals) {
        bool duplicate = false;
        for (const auto& kept : chosen) {
            if (intersectionOverSmaller(b, kept) > 0.82) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) chosen.push_back(b);
    }

    std::vector<cv::Rect> full;
    const double sx = double(right.cols) / preview.cols;
    const double sy = double(right.rows) / preview.rows;
    for (const auto& b : chosen) {
        int x0 = std::max(0, int(std::floor(b.x * sx)) - 2);
        int y0 = std::max(0, int(std::floor(b.y * sy)) - 2);
        int x1 = std::min(right.cols, int(std::ceil((b.x + b.width) * sx)) + 2);
        int y1 = std::min(right.rows, int(std::ceil((b.y + b.height) * sy)) + 2);
        if (x1 > x0 && y1 > y0)
            full.emplace_back(x0, y0, x1 - x0, y1 - y0);
    }
    std::sort(full.begin(), full.end(), [](const auto& a, const auto& b) {
        const int tolerance = std::max(10, std::min(a.height, b.height) / 4);
        if (std::abs(a.y - b.y) > tolerance) return a.y < b.y;
        return a.x < b.x;
    });
    return full;
}

// For A3: left half is resized without cropping. Right half is scanned
// for multiple independent rectangles; every output is cropped at source
// resolution and optionally resized under the existing CLI max limits.
// Existing crop|resize|combine behavior remains intact for non-A3 images.
static int splitLandscapeA3(const std::vector<std::string>& args, const cv::Mat& source) {
    const auto outputText = opt(args, "--output");
    if (outputText.empty()) throw std::runtime_error("--output is required");
    const fs::path output = utf8path(outputText);
    const int maxW = intOpt(args, "--max-width", 0);
    const int maxH = intOpt(args, "--max-height", 0);
    const int mid = source.cols / 2;
    const cv::Mat left = source(cv::Rect(0, 0, mid, source.rows));
    const cv::Mat right = source(cv::Rect(mid, 0, source.cols - mid, source.rows));
    const auto bounds = rightSideDocuments(right);
    const int resizeRight = intOpt(args, "--right-resize", 1);
    if (resizeRight > 1) throw std::runtime_error("--right-resize should be 0 or 1");

    std::vector<std::pair<fs::path, cv::Mat>> outputs;
    outputs.emplace_back(output, resizeMax(left, maxW, maxH));
    for (size_t i = 0; i < bounds.size(); ++i) {
        std::ostringstream name;
        name << output.stem().string() << "_right_" << std::setw(2)
             << std::setfill('0') << (i + 1) << output.extension().string();
        const fs::path path = output.parent_path() / fs::u8path(name.str());
        cv::Mat image = right(bounds[i]).clone();
        if (resizeRight != 0) image = resizeMax(image, maxW, maxH);
        outputs.emplace_back(path, std::move(image));
    }
    for (const auto& item : outputs)
        if (fs::exists(item.first))
            throw std::runtime_error("Output file already exists: " + item.first.u8string());

    // Delete only the files generated in this invocation if any save fails.
    std::vector<fs::path> published;
    try {
        for (const auto& item : outputs) {
            saveImage(item.first, item.second);
            published.push_back(item.first);
        }
    } catch (...) {
        for (const auto& path : published) {
            std::error_code error;
            fs::remove(path, error);
        }
        throw;
    }
    for (const auto& item : outputs)
        std::cout << "OUTPUT " << item.first.u8string() << "\n";
    if (bounds.empty()) std::cout << "WARN No independent borders found on right A4\n";
    std::cout << "OK A3_SPLIT 1 left resized; " << bounds.size() << " right crops\n";
    return 0;
}

// Resize an already detected rectangle using the same width/height semantics
// as the original PaperCrop CLI, without cropping or interpolating twice.
static cv::Mat applyCropTargets(const cv::Mat& roi, int width, int height) {
    if (width == 0 && height == 0) return roi.clone();
    const double factor = width && height
        ? std::min(double(width) / roi.cols, double(height) / roi.rows)
        : width ? double(width) / roi.cols : double(height) / roi.rows;
    int w = std::max(1, int(std::lround(roi.cols * factor)));
    int h = std::max(1, int(std::lround(roi.rows * factor)));
    cv::Mat sized;
    if (w == roi.cols && h == roi.rows) sized = roi.clone();
    else cv::resize(roi, sized, cv::Size(w,h), 0, 0,
                    (w < roi.cols || h < roi.rows) ? cv::INTER_AREA : cv::INTER_LINEAR);
    if (width && height) {
        cv::Mat canvas(height, width, roi.type(), cv::Scalar::all(255));
        sized.copyTo(canvas(cv::Rect((width-w)/2,(height-h)/2,w,h)));
        return canvas;
    }
    return sized;
}

// For normal (non-A3) scans, prefer the true photo rectangle against a
// dominant black scanner bed. On light documents continue using cropPaper,
// but reject pathological narrow slivers instead of reporting success.
static int cropWithRecovery(const std::vector<std::string>& args, const cv::Mat& source) {
    const auto output = opt(args, "--output");
    if (output.empty()) throw std::runtime_error("--output is required");
    const int targetW = intOpt(args, "--width", 0);
    const int targetH = intOpt(args, "--height", 0);
    const int maxW = intOpt(args, "--max-width", 0);
    const int maxH = intOpt(args, "--max-height", 0);
    const bool perspective = intOpt(args, "--perspective", 0) != 0;

    cv::Mat cropped;
    if (!perspective && source.cols >= 900 && source.rows >= 1100 &&
        robust::dominantLightness(source) <= 48) {
        const auto boxes = robust::darkRects(source);
        if (!boxes.empty()) {
            cropped = applyCropTargets(source(boxes.front()), targetW, targetH);
            std::cout << "INFO Dark scanner background; photo boundary recovered\n";
        }
    }
    if (cropped.empty()) {
        // First inspect the *raw* result: target-width/height white padding can
        // hide an invalid, thin crop if we inspect dimensions only after resize.
        cv::Mat raw = cropPaper(source, 0, 0, perspective);
        const double aspect = double(raw.cols) / std::max(1, raw.rows);
        const double areaRatio = double(raw.total()) / source.total();
        if (source.cols >= 700 && source.rows >= 900 &&
            robust::dominantLightness(source) > 120 &&
            (aspect < 0.36 || aspect > 4.2) && areaRatio < 0.35)
            throw std::runtime_error(
                "Crop rejected: only a narrow strip was detected; manual review required");
        if (perspective && (targetW > 0 || targetH > 0)) {
            // One resampling pass, consistent with original cropPaper.
            cropped = cropPaper(source, targetW, targetH, true);
        } else {
            cropped = applyCropTargets(raw, targetW, targetH);
        }
    }
    cropped = resizeMax(cropped, maxW, maxH);
    saveImage(utf8path(output), cropped);
    std::cout << "OK " << cropped.cols << "x" << cropped.rows << "\n";
    return 0;
}

int main(int argc, char** argv) {
    try {
        std::vector<std::string> args = getArgs(argc, argv);
        if (args.size() > 1 && args[1] == "crop") {
            const std::string mode = opt(args, "--a3-mode", "auto");
            if (mode != "off") {
                const auto input = opt(args, "--input");
                if (input.empty()) throw std::runtime_error("--input is required");
                cv::Mat source = loadImage(utf8path(input));
                if (isLandscapeA3(source, mode, utf8path(input)))
                    return splitLandscapeA3(args, source);
            }
            if (args.size() > 1 && args[1] == "crop") {
                const auto input = opt(args, "--input");
                if (input.empty()) throw std::runtime_error("--input is required");
                const cv::Mat source = loadImage(utf8path(input));
                return cropWithRecovery(args, source);
            }
        }
        return execute(args);
    } catch (const cv::Exception& ex) {
        std::cerr << "OpenCV processing error: " << ex.what() << "\n";
        return 2;
    } catch (const std::exception& ex) {
        std::cerr << "Processing error: " << ex.what() << "\n";
        return 2;
    }
}
