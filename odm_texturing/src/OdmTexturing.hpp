#pragma once

#include <iostream>
#include <fstream>
#include <string>

#ifdef HASBLENDING
    #include "opencv2/opencv_modules.hpp"
    #if defined HAVE_OPENCV_STITCHING && defined HAVE_OPENCV_NONFREE

        #include <opencv2/stitching/stitcher.hpp>
        #include "opencv2/stitching/detail/autocalib.hpp"
        #include "opencv2/stitching/detail/blenders.hpp"
        #include "opencv2/stitching/detail/camera.hpp"
        #include "opencv2/stitching/detail/exposure_compensate.hpp"
        #include "opencv2/stitching/detail/matchers.hpp"
        #include "opencv2/stitching/detail/motion_estimators.hpp"
        #include "opencv2/stitching/detail/seam_finders.hpp"
        #include "opencv2/stitching/detail/util.hpp"
        #include "opencv2/stitching/detail/warpers.hpp"
        #include "opencv2/stitching/warpers.hpp"
    #else
        #undef HASBLENDING
    #endif // defined HAVE_OPENCV_STITCHING && defined HAVE_OPENCV_NONFREE
#endif // defined HASBLENDING

// STL
#include <iostream>
#include <fstream>
#include <map>

// PCL
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/texture_mapping.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// Modified PCL functions
#include "modifiedPclFunctions.hpp"

// Logging
#include "Logger.hpp"


/*!
 * \brief The Coords struct     Coordinate class used in recursiveFindCoordinates for OdmTexturing::sortPatches().
 */
struct Coords
{
    // Coordinates for row and column
    float r_, c_;

    // If coordinates have been placed
    bool success_;

    Coords()
    {
        r_ = 0.0;
        c_ = 0.0;
        success_ = false;
    }
};

/*!
 * \brief The Patch struct      Struct to hold all faces connected and with the same optimal camera.
 */
struct Patch
{
    std::vector<size_t> faces_;
    float minu_, minv_, maxu_, maxv_;
    Coords c_;
    bool placed_;
    int materialIndex_;
    int optimalCameraIndex_;

    Patch()
    {
        placed_ = false;
        faces_ = std::vector<size_t>(0);
        minu_ = std::numeric_limits<double>::infinity();
        minv_ = std::numeric_limits<double>::infinity();
        maxu_ = 0.0;
        maxv_ = 0.0;
        optimalCameraIndex_ = -1;
        materialIndex_ = 0;
    }
};

/*!
 * \brief The Node struct       Node class for acceleration structure in OdmTexturing::sortPatches().
 */
struct Node
{
    float r_, c_, width_, height_;
    bool used_;
    Node* rgt_;
    Node* lft_;

    Node()
    {
        r_ = 0.0;
        c_ = 0.0;
        width_ = 1.0;
        height_ = 1.0;
        used_ = false;
        rgt_ = NULL;
        lft_ = NULL;
    }

    Node(const Node &n)
    {
        r_ = n.r_;
        c_ = n.c_;
        used_ = n.used_;
        width_ = n.width_;
        height_ = n.height_;
        rgt_ = n.rgt_;
        lft_ = n.lft_;
    }
};

/*!
 * \brief sortPairIndex struct  sortPairIndex class for fast sort of image quality vector.
 */
struct sortPairIndex
{
    bool operator()(const std::pair<int, int> &left, const std::pair<int, int> &right)
    {
        return left.second < right.second;
    }
};

/*!
 * \brief   The OdmTexturing class is used to create textures to a welded ply-mesh using the camera
 *          positions from pmvs as input. The result is stored in an obj-file with corresponding
 *          mtl-file and the textures saved as jpg.
 */
class OdmTexturing
{
public:
    OdmTexturing();
    ~OdmTexturing();


    /*!
     * \brief   run   Runs the texturing functionality using the provided input arguments.
     *                For a list of the accepted arguments, please see the main page documentation or
     *                call the program with parameter "-help".
     * \param   argc  Application argument count.
     * \param   argv  Argument values.
     * \return  0     if successful.
     */
    int run(int argc, char **argv);

private:

    /*!
     * \brief parseArguments    Parses command line arguments.
     * \param argc              Application argument count.
     * \param argv              Argument values.
     */
    void parseArguments(int argc, char** argv);

    /*!
     * \brief loadMesh          Loads a PLY-file containing vertices and faces.
     */
    void loadMesh();

    /*!
     * \brief loadCameras       Loads cameras from a bundle.out file with corresponding image list file.
     */
    void loadCameras();

    /*!
     * \brief blendImages           Blends each image with its k neares neighbors.
     */
    void blendImages();

    /*!
     * \brief loadCameras           Blends each image with its k neares neighbors.
     *
     * \param image                 The image to be compensated
     * \param currentCameraIndex    index in cameras_ vector of the camera to be blended
     */
    void blendImage(cv::Mat &image, size_t currentCameraIndex);

    /*!
    * \brief evaluateImageQuality     Evaluates and sorts the images according to quality regarding blur, over/under exposure and alignment.
    */
    void evaluateImageQuality();

    /*!
     * \brief triangleToImageAssignment     Assigns optimal camera to faces for the faces that are visible.
     */
    void triangleToImageAssignment();

    /*!
     * \brief calculatePatches      Arrange faces into patches as a prestep to arranging UV-mapping.
     */
    void calculatePatches();

    /*!
     * \brief recursiveFindCoords   Recursive function used in sortPatches() to find free area to place patch.
     * \param n                     The container in which to check for free space in.
     * \param w                     The width of the box to place.
     * \param h                     The height of the box to place.
     * \return                      The coordinates where the patch has been placed.
     */
    Coords recursiveFindCoords(Node &n, float w, float h);

    /*!
     * \brief sortPatches       Sorts patches into UV-containers to be used in createTextures() using a rectangle packer approach.
     */
    void sortPatches();

    /*!
     * \brief createTextures    Creates textures to the mesh.
     */
    void createTextures();

    /*!
     * \brief writeObjFile      Writes the textured mesh to file on the OBJ format.
     */
    void writeObjFile();

    /*!
     * \brief affine3fToCvMat4x4    Converts an Eigen 4x4 matrix to an OpenCV 4x4 matrix.
     *
     * \param   input   Input Eigen 4x4 Matrix
     *
     * \return  input matrix as an OpenCV matrix.
     */
    cv::Mat affine3fToCvMat4x4(const Eigen::Affine3f &input);


    /*!
     * \brief printHelp         Prints help, explaining usage. Can be shown by calling the program with arguments: "-help".
     */
    void printHelp();

    Logger log_;                    /**< Logging object. */
    std::string logFilePath_;       /**< Path to store the log file. */

    std::string bundlePath_;                    /**< Path to the bundle.out file. */
    std::string imagesPath_;                    /**< Path to the folder with all images in the image list. */
    std::string imagesListPath_;                /**< Path to the image list. */
    std::string inputModelPath_;                /**< Path to the ply-file containing the mesh to be textured. */
    std::string outputFolder_;                  /**< Path to the folder to store the output mesh and textures. */
    std::string outputBlendedImagesFolder_;     /**< Path to the folder where blended/compensated images are stored. */

    double bundleResizedTo_;        /**< The size used in the previous steps to calculate the camera focal_length. */
    double textureWithSize_;        /**< The desired size of the images to texture with. */
    double textureResolution_;      /**< The resolution of each texture. */
    double padding_;                /**< A padding used to handle edge cases. */
    int nrTextures_;                /**< The number of textures created. */
    int nrCamerasToBlend_;          /**< The number of camera pairs used when blending/exposure compensating each image. If <= 0, no blending/exposure compensation is performed. */
    bool performBlending_;           /**< If true, blending is performed (provided that nrCamerasToBlend_ > 0). */

    pcl::TextureMesh::Ptr mesh_;                    /**< PCL Texture Mesh */
    std::vector<Patch> patches_;                    /**< The vector containing all patches */
    std::vector<std::vector<Patch> > patchesVec_;   /**< The vector containing all possible patches */
    pcl::texture_mapping::CameraVector cameras_;    /**< The vector containing all cameras. */
    std::vector<int> tTIA_;                         /**< The vector containing the optimal cameras for all faces. */
    std::vector<int> sortedCameras_;                /**< The vector containing the cameras in sorted order after a quality index. */
};

class OdmTexturingException : public std::exception
{

public:
    OdmTexturingException() : message("Error in OdmTexturing") {}
    OdmTexturingException(std::string msgInit) : message("Error in OdmTexturing:\n" + msgInit) {}
    ~OdmTexturingException() throw() {}
    virtual const char* what() const throw() {return message.c_str(); }

private:
    std::string message;        /**< The error message. */
};
