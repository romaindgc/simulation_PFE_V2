#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <iostream>

namespace image_utils {

inline std::string generate_image_filename(const std::string& base_path, const std::string& prefix = "image", const std::string& extension = ".png") {
    static int image_counter = 0;  // compteur global à cette fonction

    // Récupérer le timestamp actuel
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::stringstream ss;
    ss << base_path << "/" << prefix << "_"
       << std::put_time(&local_tm, "%Y%m%d_%H%M%S")  // format horodaté
       << "_" << image_counter++
       << extension;

    return ss.str();
}

inline std::string generate_imagette_filename(const std::string& base_path, const std::string& prefix = "imagette", const std::string& extension = ".png") {
    static int image_counter = 0;  // compteur global à cette fonction

    // Récupérer le timestamp actuel
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm = *std::localtime(&now_c);

    std::stringstream ss;
    ss << base_path << "/" << prefix << "_"
       << std::put_time(&local_tm, "%Y%m%d_%H%M%S")  // format horodaté
       << "_" << image_counter++
       << extension;

    return ss.str();
}

/// Calcule le score de netteté Tenengrad
inline double tenengrad(const cv::Mat& img, int kernel_size = 5) {
    cv::Mat sx, sy;
    cv::Sobel(img, sx, CV_32F, 1, 0, kernel_size);
    cv::Sobel(img, sy, CV_32F, 0, 1, kernel_size);
    cv::Mat magnitude;
    cv::magnitude(sx, sy, magnitude);
    return cv::mean(magnitude)[0];
}

/// Alias pour Tenengrad
inline double sharpness_score(const cv::Mat& img, int kernel_size = 5) {
    return tenengrad(img, kernel_size);
}

/// Détection de cercles après binarisation de l’image (renvoie un vecteur de cercles : (x, y, r))
inline std::vector<cv::Vec3f> circle_detection_binarized(
    const cv::Mat& img,
    double dp = 1.2,
    double min_dist = 20,
    double param1 = 100,
    double param2 = 15,
    int min_radius = 5,
    int max_radius = 30
) {
    std::vector<cv::Vec3f> detected_circles;

    if (img.empty()) {
        std::cerr << "Erreur : image vide dans circle_detection_binarized." << std::endl;
        return detected_circles;
    }

    // Convertir en niveaux de gris si nécessaire
    cv::Mat gray;
    if (img.channels() == 3)
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    else
        gray = img.clone();

    // Vérifier profondeur
    if (gray.depth() != CV_8U) {
        double minVal, maxVal;
        cv::minMaxLoc(gray, &minVal, &maxVal);
        gray.convertTo(gray, CV_8U, 255.0 / (maxVal > 0 ? maxVal : 1.0));
    }

    // Binarisation
    cv::Mat img_thresh;
    cv::threshold(gray, img_thresh, 50, 255, cv::THRESH_BINARY_INV);
    cv::imwrite("/home/iris2/dev/debug_threshold.png", img_thresh);

    // HoughCircles dans un vecteur (clé du bug résolu ici)
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(
        img_thresh,
        circles,
        cv::HOUGH_GRADIENT,
        dp,
        min_dist,
        param1,
        param2,
        min_radius,
        max_radius
    );

    // Affichage debug
    std::cerr << "Cercles détectés : " << circles.size() << std::endl;

    // Visualisation
    cv::Mat output;
    cv::cvtColor(gray, output, cv::COLOR_GRAY2BGR);

    for (size_t i = 0; i < circles.size(); ++i) {
        float x = circles[i][0];
        float y = circles[i][1];
        float r = circles[i][2];

        int x_i = static_cast<int>(std::round(x));
        int y_i = static_cast<int>(std::round(y));
        int r_i = static_cast<int>(std::round(r));

        cv::circle(output, cv::Point(x_i, y_i), r_i, cv::Scalar(0, 255, 0), 2);
        cv::circle(output, cv::Point(x_i, y_i), 1, cv::Scalar(0, 0, 255), 3);

        detected_circles.emplace_back(x, y, r);
    }

    cv::imwrite("/home/iris2/dev/debug_output.png", output);
    return detected_circles;
}



/// Extrait une imagette centrée sur `centre` de dimensions (largeur, hauteur)
inline cv::Mat extraire_imagette(const cv::Mat& img, cv::Point centre, int largeur, int hauteur) {
    int x = centre.x;
    int y = centre.y;
    int demi_l = largeur / 2;
    int demi_h = hauteur / 2;

    int x1 = std::max(0, x - demi_l);
    int y1 = std::max(0, y - demi_h);
    int x2 = std::min(img.cols, x + demi_l);
    int y2 = std::min(img.rows, y + demi_h);

    if (x1 >= x2 || y1 >= y2) {
        std::cerr << "Erreur : coordonnées invalides pour l'extraction." << std::endl;
        return cv::Mat();  // image vide
    }

    return img(cv::Rect(x1, y1, x2 - x1, y2 - y1)).clone();
}

}  // namespace image_utils
