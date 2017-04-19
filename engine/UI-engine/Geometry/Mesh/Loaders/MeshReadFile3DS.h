#ifndef MESHREADFILE3DS_H
#define MESHREADFILE3DS_H

#include "../Mesh.h"


namespace utility_engine
{


    namespace
    {
        const int MAIN3DS = 0x4D4D; //<a> *.3DS
        const int EDIT3DS = 0x3D3D;
        const int EDIT_OBJECT = 0x4000;
        //------ sub defines of EDIT_OBJECT
        const int OBJ_TRIMESH = 0x4100;
        //------ sub defines of OBJ_TRIMESH
        const int TRI_VERTEXLIST = 0x4110;
        const int TRI_VERTEXOPTIONS = 0x4111;
        const int TRI_MAPPINGCOORS = 0x4140;
        const int TRI_MAPPINGSTANDARD = 0x4170;
        const int TRI_FACELIST = 0x4120;
        const int TRI_SMOOTH = 0x4150;
        const int TRI_MATERIAL = 0x4130;
        const int TRI_LOCAL = 0x4160;
        const int TRI_VISIBLE = 0x4165;

    }



    class MeshReadFile3DS : public Mesh
    {
      public:

    	MeshReadFile3DS(const char *fileName);


     private:

       void initilisation( const char* fileName );



       static bool Load3DSFile(const char *fileName,   std::vector<Vector3>  &vertices ,
                                                       std::vector<Vector3>  &normals  ,
                                                       std::vector<Vector2>  &vtxUV    ,
                                                       std::vector<uint>     &indicess ,
                                                       std::vector<std::vector<uint> > &indices  ,
                                                       std::vector<Color>    &Colors ,
                                                       bool invertTransform = true );

       static unsigned int FindChunk(std::ifstream & ifs, unsigned short  id, bool isParent = true );
    };

}

#endif // MESHREADFILE3DS_H
