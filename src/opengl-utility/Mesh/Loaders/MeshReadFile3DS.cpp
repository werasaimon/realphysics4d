#include "MeshReadFile3DS.h"

#include <fstream>
#include <iostream>

using namespace std;
using namespace opengl_utility;


MeshReadFile3DS::MeshReadFile3DS(const char *fileName)
: Mesh(MESH_LOAD_FILE)
{
    initilisation(fileName);
}

void MeshReadFile3DS::initilisation(const char *fileName)
{
    if(Load3DSFile( fileName , this->mVertices , this->mNormals , this->mUVs , this->mIndicess ));
}



bool MeshReadFile3DS::Load3DSFile(const char *fileName,
                                  std::vector<Vector3> &vertices,
                                  std::vector<Vector3> &normals,
                                  std::vector<Vector2> &vtxUV,
                                  std::vector<uint> &indices, bool invertTransform)
{

    unsigned short chunk_id;
    unsigned int   chunk_pos;
    unsigned int   chunk_temppos;
    unsigned int   chunk_len;

    unsigned short nVertexs = 0;

    ifstream ifs;
    ifs.open(fileName, ios_base::in |
            ios_base::binary |
            ios_base::openmode(ios_base::beg));


    if (!ifs.is_open())
    {
        cout << "not File:: <"<< fileName <<">"<< endl;
        return false;
    }
    else
    {

        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);
        if (chunk_id != MAIN3DS) return false;
        ifs.seekg(ifs.tellg() - (std::streamoff) 6);

        chunk_pos = FindChunk(ifs, EDIT3DS);
        if (chunk_pos == 0)  return false;




        chunk_pos = FindChunk(ifs, EDIT_OBJECT);
        if (chunk_pos == 0)  return false;

        chunk_pos = FindChunk(ifs, OBJ_TRIMESH);
        if (chunk_pos == 0) return false;

        chunk_temppos = chunk_pos;

        chunk_pos = FindChunk(ifs, 0x4110);
        if (chunk_pos == 0) return false;
        ifs.ignore(6);


        ifs.read((char*) &nVertexs, 2);

        vertices.resize(nVertexs);
        for (int i = 0; i < nVertexs; ++i)
        {
            float x,y,z;
            ifs.read((char*) &(x), 4);
            ifs.read((char*) &(y), 4);	        //y � z
            ifs.read((char*) &(z), 4);

            vertices[i] = Vector3(x,y,z);

        }



    }


    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_MAPPINGCOORS);
    if (chunk_pos == 0) return false;
    ifs.ignore(6);

    unsigned short nTexCoords;
    ifs.read((char*) &nTexCoords, 2);


    vtxUV.resize(nTexCoords);
    for (int i = 0; i < nTexCoords; i++)
    {
        float x;
        float y;
        ifs.read((char*) &(x), 4);
        ifs.read((char*) &(y), 4);

        vtxUV[i] = Vector2(x,y);
    }




    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_FACELIST);
    if (chunk_pos == 0)  return false;
    ifs.ignore(6);

    unsigned short nTriangles;
    ifs.read((char*) &nTriangles, 2);


     normals.resize(nVertexs);
    //indices.resize( nTriangles * 3 );
    for (int i = 0; i < nTriangles; i++)
    {
        unsigned short A, B, C;
        ifs.read((char*) &(A), 2);
        ifs.read((char*) &(B), 2);
        ifs.read((char*) &(C), 2);

        indices.push_back(A);
        indices.push_back(B);
        indices.push_back(C);

        Vector3 p = vertices[A];
        Vector3 q = vertices[B];
        Vector3 r = vertices[C];

        Vector3 N = ((q-p).cross(r-p)).normalize();


        normals[A] += N;
        normals[B] += N;
        normals[C] += N;
        /*********************************************
        vec3 p = vertices[A].position;
        vec3 q = vertices[B].position;
        vec3 r = vertices[C].position;
        vec3 normal = normalize( cross( (q-p) , (r-p)) );
        vertices[A].normal += normal;
        vertices[B].normal += normal;
        vertices[C].normal += normal;
        /**********************************************/

        ifs.ignore(2);

    }

    std::cout<< "  load file:   "<< fileName << "  "<< "OK!!"  <<std::endl;

    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_LOCAL);
    if (chunk_pos == 0) return false;
    ifs.ignore(6);


    float Local[12] = { 0.0f };
    ifs.read((char*) &Local, sizeof(float) * 12);


//					wsVector3   pos(Local[9], Local[11], Local[10]);
//					wsMatrix3x3 rot(Local[0], Local[2], Local[1],
//							        Local[6], Local[8], Local[7],
//						        	Local[3], Local[5], Local[4]);


//				if ( invertTransform )
//				{
//					wsTransform   transform( pos , rot );
//					for (int i = 0; i < (int)vertices.size(); ++i)
//					{
//						wsVector3 point;
//						point.x = vertices[i].position.x;
//						point.y = vertices[i].position.y;
//						point.z = vertices[i].position.z;
//
//						wsVector3 invVertex = ( point  - pos ) ^ rot;
//
//						vertices[i].position.x = invVertex.x;
//						vertices[i].position.y = invVertex.y;
//						vertices[i].position.z = invVertex.z;
//					}
//				}
//				else
//				{
//					//wsTransform   transform( pos , rot );
//					//m_transform = transform;
//				}



    ifs.seekg(chunk_temppos);
    chunk_pos = FindChunk(ifs, TRI_MATERIAL);
    if (chunk_pos == 0)  return false;
    ifs.ignore(6);


    return true;
}


unsigned int MeshReadFile3DS::FindChunk(ifstream &ifs, unsigned short id, bool isParent)
{
    unsigned short chunk_id;
    unsigned int   chunk_len;

    if (isParent)
    {
        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);

        if (chunk_id == EDIT_OBJECT)
        {
            char ch;
            do
            {
                ifs.read((char*) &ch, 1);
            } while (ch != '\0' && !ifs.eof());
        }
    }
    do
    {
        ifs.read((char*) &chunk_id, 2);
        ifs.read((char*) &chunk_len, 4);

        if (chunk_id != id)
        {
            ifs.ignore(chunk_len - 6);
        }
        else
        {
            ifs.seekg(ifs.tellg() - (std::streamoff) 6);
            return (ifs.tellg());
        }
    }
    while (!ifs.eof());

    return 0;
}
