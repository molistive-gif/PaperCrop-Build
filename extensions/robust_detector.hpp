#pragma once
// Conservative recovery detector for scanner images. OpenCV C++ is the only
// image-processing runtime dependency. No private production photos are stored
// or shipped with this header.
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <vector>

namespace robust {
struct Dpi { double x = 0; double y = 0; };

// JFIF stores the scanner resolution in its APP0 header. Extract it without
// depending on OpenCV's metadata API (not available in OpenCV 4.9).
inline Dpi readJfifDpi(const std::filesystem::path& filename) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) return {};
    std::vector<unsigned char> bytes(65536);
    f.read(reinterpret_cast<char*>(bytes.data()), std::streamsize(bytes.size()));
    bytes.resize(static_cast<size_t>(f.gcount()));
    if (bytes.size() < 18 || bytes[0] != 0xFF || bytes[1] != 0xD8) return {};
    size_t pos = 2;
    while (pos + 5 < bytes.size()) {
        if (bytes[pos] != 0xFF) { ++pos; continue; }
        size_t p = pos + 1;
        while (p < bytes.size() && bytes[p] == 0xFF) ++p;
        if (p + 3 >= bytes.size()) break;
        unsigned char marker = bytes[p];
        if (marker == 0xDA || marker == 0xD9) break; // start-of-scan / end
        if (marker == 0x01 || (marker >= 0xD0 && marker <= 0xD7)) {
            pos = p + 1; continue;
        }
        unsigned int length = (unsigned(bytes[p + 1]) << 8) | bytes[p + 2];
        size_t payload = p + 3;
        if (length < 2 || payload + length - 2 > bytes.size()) break;
        if (marker == 0xE0 && length >= 16 &&
            bytes[payload] == 'J' && bytes[payload+1] == 'F' &&
            bytes[payload+2] == 'I' && bytes[payload+3] == 'F' &&
            bytes[payload+4] == 0) {
            int units = bytes[payload + 7];
            unsigned int xd = (unsigned(bytes[payload+8]) << 8) | bytes[payload+9];
            unsigned int yd = (unsigned(bytes[payload+10]) << 8) | bytes[payload+11];
            if (xd && yd && (units == 1 || units == 2)) {
                double factor = (units == 2) ? 2.54 : 1.0;
                return {xd * factor, yd * factor};
            }
        }
        pos = p + 1 + length;
    }
    return {};
}

inline int dominantLightness(const cv::Mat& image) {
    if (image.empty()) return 255;
    const int x0 = std::max(0, image.cols * 7 / 100);
    const int x1 = std::max(x0 + 1, image.cols * 93 / 100);
    const int y0 = std::max(0, image.rows * 7 / 100);
    const int y1 = std::max(y0 + 1, image.rows * 93 / 100);
    std::array<int, 256> h{};
    int sampled = 0;
    const int stride = std::max(1, std::min(image.cols, image.rows) / 250);
    for (int y = y0; y < y1; y += stride) {
        const cv::Vec3b* row = image.ptr<cv::Vec3b>(y);
        for (int x = x0; x < x1; x += stride) {
            const auto& c = row[x];
            ++h[std::max({int(c[0]), int(c[1]), int(c[2])})];
            ++sampled;
        }
    }
    int cumulative = 0;
    for (int i = 0; i < 256; ++i) {
        cumulative += h[i];
        if (cumulative > sampled / 2) return i;
    }
    return 255;
}
struct Candidate {
    cv::Rect box;
    double score = 0;
    double fill = 0;
};
inline double intersectionRatio(const cv::Rect& a, const cv::Rect& b) {
    double denom = double(std::min(a.area(), b.area()));
    return denom > 0 ? double((a & b).area()) / denom : 0;
}

// Finds genuinely independent photo/document rectangles on DARK scanner beds.
// Handles photographs in any location, including small 3x4 portraits and
// photographs close to the scanner edge. Returns source-resolution rectangles.
inline std::vector<cv::Rect> darkRects(const cv::Mat& source) {
    if (source.empty() || source.channels() != 3) return {};
    const double scale = std::min(1.0, 1600.0 / std::max(source.cols, source.rows));
    cv::Mat preview;
    if (scale < 1.0)
        cv::resize(source, preview, cv::Size(), scale, scale, cv::INTER_AREA);
    else
        preview = source;
    const int background = dominantLightness(preview);
    if (background > 48) return {};

    std::vector<cv::Mat> channels;
    cv::split(preview, channels);
    cv::Mat light;
    cv::max(channels[0], channels[1], light);
    cv::max(light, channels[2], light);
    cv::Mat mask;
    cv::threshold(light, mask, std::max(33, background + 24), 255, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_RECT, {5, 5}));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));

    std::vector<Candidate> proposed;
    auto add = [&](const cv::Mat& binary, int retrieval) {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary.clone(), contours, retrieval, cv::CHAIN_APPROX_SIMPLE);
        const double total = double(preview.cols) * preview.rows;
        for (const auto& c : contours) {
            if (c.size() < 4) continue;
            cv::Rect b = cv::boundingRect(c);
            if (b.width < 18 || b.height < 18) continue;
            const double fraction = double(b.area()) / total;
            if (fraction < 0.00013 || fraction > 0.50) continue;
            const double aspect = double(b.width) / b.height;
            if (aspect < 0.23 || aspect > 4.3) continue;
            const double fill = cv::contourArea(c) / std::max(1, b.area());
            if (fill < 0.25) continue;
            const bool touchesFrame = b.x < 4 || b.y < 4 ||
                b.br().x >= preview.cols - 4 || b.br().y >= preview.rows - 4;
            const double score = b.area() * std::pow(std::min(1.0, fill), 0.7) *
                (touchesFrame ? 0.24 : 1.0);
            proposed.push_back({b, score, fill});
        }
    };
    add(mask, cv::RETR_EXTERNAL);

    // A light scanner streak can join a photo to an entire image border.
    // In that case the brightness component becomes a thin, invalid contour.
    // Recover the photo from its local rectangular edges instead.
    if (proposed.empty()) {
        cv::Mat gray, edges;
        cv::cvtColor(preview, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, {5, 5}, 0);
        cv::Canny(gray, edges, 35, 105);
        cv::dilate(edges, edges,
            cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
        cv::morphologyEx(edges, edges, cv::MORPH_CLOSE,
            cv::getStructuringElement(cv::MORPH_RECT, {7, 7}));
        add(edges, cv::RETR_LIST);
    }
    if (proposed.empty()) return {};

    // The same photo often has multiple contours (white frame, face, etc.).
    // Merge candidates with substantial overlap, never merge disjoint photos.
    std::sort(proposed.begin(), proposed.end(), [](const auto& a, const auto& b) {
        return a.score > b.score;
    });
    std::vector<Candidate> accepted;
    for (const auto& c : proposed) {
        bool duplicate = false;
        for (auto& k : accepted) {
            if (intersectionRatio(c.box, k.box) > 0.52) {
                // Border artefacts get a low score, so prefer the inner photo.
                // If the inner outlines partially overlap, union them to keep
                // the complete photo rather than a face-only crop.
                if (c.score > k.score * 0.70 && c.fill > 0.35) {
                    cv::Rect joined = c.box | k.box;
                    if (joined.area() < 2.2 * std::max(c.box.area(), k.box.area()))
                        k.box = joined;
                }
                duplicate = true;
                break;
            }
        }
        if (!duplicate) accepted.push_back(c);
    }
    std::vector<cv::Rect> full;
    const double sx = double(source.cols) / preview.cols;
    const double sy = double(source.rows) / preview.rows;
    const cv::Rect frame(0, 0, source.cols, source.rows);
    for (const auto& c : accepted) {
        const auto& b = c.box;
        cv::Rect r(
            int(std::floor(b.x * sx)) - 3,
            int(std::floor(b.y * sy)) - 3,
            int(std::ceil(b.width * sx)) + 6,
            int(std::ceil(b.height * sy)) + 6);
        r &= frame;
        if (r.area() > 0) full.push_back(r);
    }
    return full;
}
} // namespace robust
