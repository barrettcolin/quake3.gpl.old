#include <float.h>

extern "C"
{
#include "tr_local.h"
}

/*
=================
R_MirrorPoint
=================
*/
static void R_MirrorPoint (vec3_t in, orientation_t *surface, orientation_t *camera, vec3_t out)
{
    int		i;
    vec3_t	local;
    vec3_t	transformed;
    float	d;

    VectorSubtract( in, surface->origin, local );

    VectorClear( transformed );
    for ( i = 0 ; i < 3 ; i++ ) {
        d = DotProduct(local, surface->axis[i]);
        VectorMA( transformed, d, camera->axis[i], transformed );
    }

    VectorAdd( transformed, camera->origin, out );
}

static void R_MirrorVector (vec3_t in, orientation_t *surface, orientation_t *camera, vec3_t out)
{
    int		i;
    float	d;

    VectorClear( out );
    for ( i = 0 ; i < 3 ; i++ ) {
        d = DotProduct(in, surface->axis[i]);
        VectorMA( out, d, camera->axis[i], out );
    }
}

// Generate a plane given a surfType
static void PlaneForSurface(surfaceType_t const *const surfType, cplane_t& plane)
{
    vec4_t plane4 = { 0, 0, 0, 1 };

    if(surfType)
    {
        surfaceType_t const st = *surfType;

        switch(st)
        {
        case SF_FACE:
            plane = ((srfSurfaceFace_t *)surfType)->plane;
            return;
        case SF_TRIANGLES:
            {
                srfTriangles_t const *const tri = (srfTriangles_t *)surfType;
                PlaneFromPoints(plane4, 
                    (tri->verts + tri->indexes[0])->xyz, 
                    (tri->verts + tri->indexes[1])->xyz, 
                    (tri->verts + tri->indexes[2])->xyz);
            }
            break;
        case SF_POLY:
            {
                srfPoly_t const *const poly = (srfPoly_t *)surfType;
                PlaneFromPoints(plane4, 
                    poly->verts[0].xyz, 
                    poly->verts[1].xyz, 
                    poly->verts[2].xyz);
            }
            break;
        }
    }

    VectorCopy(plane4, plane.normal);
    plane.dist = plane4[3];
}

/*
=================
R_GetPortalOrientation

entityNum is the entity that the portal surface is a part of, which may
be moving and rotating.

Returns qtrue if it should be mirrored
=================
*/
// surface->origin is a point on the portal plane
// surface->axis[0] is the portal plane normal

static bool GetPortalOrientations(int const entityNum,
                                  cplane_t const& portalPlane, 
                                  orientation_t *const surface, 
                                  orientation_t *const camera,
                                  vec3_t& pvsOrigin, 
                                  bool& mirror )
{
    cplane_t	originalPlane, plane;
    vec3_t		transformed;

    originalPlane = portalPlane;

    // rotate the plane if necessary
    if ( entityNum != ENTITYNUM_WORLD ) {
        tr.currentEntityNum = entityNum;
        tr.currentEntity = &tr.refdef.entities[entityNum];

        // get the orientation of the entity
        R_RotateForEntity( tr.currentEntity, &tr.viewParms, &tr.or );

        // rotate the plane, but keep the non-rotated version for matching
        // against the portalSurface entities
        R_LocalNormalToWorld( originalPlane.normal, plane.normal );
        plane.dist = originalPlane.dist + DotProduct( plane.normal, tr.or.origin );

        // translate the original plane
        originalPlane.dist = originalPlane.dist + DotProduct( originalPlane.normal, tr.or.origin );
    } else {
        plane = originalPlane;
    }

    // surface->axis[0] = plane.normal
    // surface->axis[1] = perpendicular vector
    // surface->axis[2] = cross product
    VectorCopy(plane.normal, surface->axis[0]);
    PerpendicularVector(surface->axis[1], surface->axis[0]);
    CrossProduct(surface->axis[0], surface->axis[1], surface->axis[2]);

    // locate the portal entity closest to this plane.
    // origin will be the origin of the portal, origin2 will be
    // the origin of the camera
    for(int i = 0 ; i < tr.refdef.num_entities; i++)
    {
        trRefEntity_t const *const e = &tr.refdef.entities[i];
        if(e->e.reType != RT_PORTALSURFACE)
            continue;

        float d = DotProduct(e->e.origin, originalPlane.normal) - originalPlane.dist;
        if(d > 64 || d < -64)
            continue;

        // get the pvsOrigin from the entity
        VectorCopy(e->e.oldorigin, pvsOrigin);

        // if the entity is just a mirror, don't use as a camera point
        if(VectorCompare(e->e.oldorigin, e->e.origin))
        {
            // Need camera parameters to set up clip plane:
            // camera->origin = plane.normal * plane.dist
            // camera->axis[0] = -surface->axis[0]
            VectorScale(plane.normal, plane.dist, camera->origin);
            VectorSubtract(vec3_origin, surface->axis[0], camera->axis[0]);

            mirror = true;
            return true;
        }

        // project the origin onto the surface plane to get
        // an origin point we can rotate around
        d = DotProduct(e->e.origin, plane.normal) - plane.dist;
        // surface->origin = e->e.origin - d * surface->axis[0]
        VectorMA(e->e.origin, -d, surface->axis[0], surface->origin);

        // now get the camera origin and orientation
        VectorCopy(e->e.oldorigin, camera->origin);
        VectorSubtract(vec3_origin, e->e.axis[0], camera->axis[0]);
        VectorSubtract(vec3_origin, e->e.axis[1], camera->axis[1]);
        VectorCopy(e->e.axis[2], camera->axis[2]);

        // optionally rotate
        if ( e->e.oldframe ) {
            // if a speed is specified
            if ( e->e.frame ) {
                // continuous rotate
                d = (tr.refdef.time/1000.0f) * e->e.frame;
                VectorCopy( camera->axis[1], transformed );
                RotatePointAroundVector( camera->axis[1], camera->axis[0], transformed, d );
                CrossProduct( camera->axis[0], camera->axis[1], camera->axis[2] );
            } else {
                // bobbing rotate, with skinNum being the rotation offset
                d = sin( tr.refdef.time * 0.003f );
                d = e->e.skinNum + d * 4;
                VectorCopy( camera->axis[1], transformed );
                RotatePointAroundVector( camera->axis[1], camera->axis[0], transformed, d );
                CrossProduct( camera->axis[0], camera->axis[1], camera->axis[2] );
            }
        }
        else if ( e->e.skinNum )
        {
            d = e->e.skinNum;
            VectorCopy( camera->axis[1], transformed );
            RotatePointAroundVector( camera->axis[1], camera->axis[0], transformed, d );
            CrossProduct( camera->axis[0], camera->axis[1], camera->axis[2] );
        }

        mirror = false;
        return true;
    }

    // if we didn't locate a portal entity, don't render anything.
    // We don't want to just treat it as a mirror, because without a
    // portal entity the server won't have communicated a proper entity set
    // in the snapshot

    // unfortunately, with local movement prediction it is easily possible
    // to see a surface before the server has communicated the matching
    // portal surface entity, so we don't want to print anything here...

    //ri.Printf( PRINT_ALL, "Portal surface without a portal entity\n" );

    return false;
}
#if 0
static qboolean IsMirror( const drawSurf_t *drawSurf, int entityNum )
{
    int			i;
    cplane_t	originalPlane, plane;
    trRefEntity_t	*e;
    float		d;

    // create plane axis for the portal we are seeing
    R_PlaneForSurface( drawSurf->surface, &originalPlane );

    // rotate the plane if necessary
    if ( entityNum != ENTITYNUM_WORLD ) 
    {
        tr.currentEntityNum = entityNum;
        tr.currentEntity = &tr.refdef.entities[entityNum];

        // get the orientation of the entity
        R_RotateForEntity( tr.currentEntity, &tr.viewParms, &tr.or );

        // rotate the plane, but keep the non-rotated version for matching
        // against the portalSurface entities
        R_LocalNormalToWorld( originalPlane.normal, plane.normal );
        plane.dist = originalPlane.dist + DotProduct( plane.normal, tr.or.origin );

        // translate the original plane
        originalPlane.dist = originalPlane.dist + DotProduct( originalPlane.normal, tr.or.origin );
    } 
    else 
    {
        plane = originalPlane;
    }

    // locate the portal entity closest to this plane.
    // origin will be the origin of the portal, origin2 will be
    // the origin of the camera
    for ( i = 0 ; i < tr.refdef.num_entities ; i++ ) 
    {
        e = &tr.refdef.entities[i];
        if ( e->e.reType != RT_PORTALSURFACE ) {
            continue;
        }

        d = DotProduct( e->e.origin, originalPlane.normal ) - originalPlane.dist;
        if ( d > 64 || d < -64) {
            continue;
        }

        // if the entity is just a mirror, don't use as a camera point
        if ( e->e.oldorigin[0] == e->e.origin[0] && 
            e->e.oldorigin[1] == e->e.origin[1] && 
            e->e.oldorigin[2] == e->e.origin[2] ) 
        {
            return qtrue;
        }

        return qfalse;
    }
    return qfalse;
}

/*
** SurfIsOffscreen
**
** Determines if a surface is completely offscreen.
*/
static qboolean SurfIsOffscreen( const drawSurf_t *drawSurf, vec4_t clipDest[128] )
{
    float shortest = 100000000;
    int entityNum;
    int numTriangles;
    shader_t *shader;
    int		fogNum;
    int dlighted;
    vec4_t clip, eye;
    int i;
    unsigned int pointOr = 0;
    unsigned int pointAnd = (unsigned int)~0;

    if ( glConfig.smpActive ) {		// FIXME!  we can't do RB_BeginSurface/RB_EndSurface stuff with smp!
        return qfalse;
    }

    R_RotateForViewer();

    R_DecomposeSort( drawSurf->sort, &entityNum, &shader, &fogNum, &dlighted );
    RB_BeginSurface( shader, fogNum );
    rb_surfaceTable[ *drawSurf->surface ]( drawSurf->surface );

    assert( tess.numVertexes < 128 );

    for ( i = 0; i < tess.numVertexes; i++ )
    {
        int j;
        unsigned int pointFlags = 0;

        R_TransformModelToClip( tess.xyz[i], tr.or.modelMatrix, tr.viewParms.projectionMatrix, eye, clip );

        for ( j = 0; j < 3; j++ )
        {
            if ( clip[j] >= clip[3] )
            {
                pointFlags |= (1 << (j*2));
            }
            else if ( clip[j] <= -clip[3] )
            {
                pointFlags |= ( 1 << (j*2+1));
            }
        }
        pointAnd &= pointFlags;
        pointOr |= pointFlags;
    }

    // trivially reject
    if ( pointAnd )
    {
        return qtrue;
    }

    // determine if this surface is backfaced and also determine the distance
    // to the nearest vertex so we can cull based on portal range.  Culling
    // based on vertex distance isn't 100% correct (we should be checking for
    // range to the surface), but it's good enough for the types of portals
    // we have in the game right now.
    numTriangles = tess.numIndexes / 3;

    for ( i = 0; i < tess.numIndexes; i += 3 )
    {
        vec3_t normal;
        float dot;
        float len;

        VectorSubtract( tess.xyz[tess.indexes[i]], tr.viewParms.or.origin, normal );

        len = VectorLengthSquared( normal );			// lose the sqrt
        if ( len < shortest )
        {
            shortest = len;
        }

        if ( ( dot = DotProduct( normal, tess.normal[tess.indexes[i]] ) ) >= 0 )
        {
            numTriangles--;
        }
    }
    if ( !numTriangles )
    {
        return qtrue;
    }

    // mirrors can early out at this point, since we don't do a fade over distance
    // with them (although we could)
    if ( IsMirror( drawSurf, entityNum ) )
    {
        return qfalse;
    }

    if ( shortest > (tess.shader->portalRange*tess.shader->portalRange) )
    {
        return qtrue;
    }

    return qfalse;
}
#endif
/*
========================
R_MirrorViewBySurface

Returns qtrue if another view has been rendered
========================
*/
qboolean R_MirrorViewBySurface(drawSurf_t const *const drawSurf, int const entityNum)
{
    // don't recursively mirror
    if(tr.viewParms.isPortal)
    {
        ri.Printf(PRINT_DEVELOPER, "WARNING: recursive mirror/portal found\n");
        return qfalse;
    }

    if(r_noportals->integer || (r_fastsky->integer == 1))
        return qfalse;

#if 0
    // trivially reject portal/mirror
    if ( SurfIsOffscreen( drawSurf, clipDest ) ) {
        return qfalse;
    }
#endif

    cplane_t portalPlane;
    PlaneForSurface(drawSurf->surface, portalPlane);

    orientation_t surface, camera;
    vec3_t pvs_origin;
    bool is_mirror;
    if(!GetPortalOrientations(entityNum, portalPlane, &surface, &camera, pvs_origin, is_mirror))
    {
        // bad portal, no portalentity
        return qfalse;
    }

    // Set up new viewParms
    viewParms_t newParms = tr.viewParms;
    {
        newParms.isPortal = qtrue;
        VectorCopy(pvs_origin, newParms.pvsOrigin);
        if(is_mirror)
        {
            newParms.isMirror = qtrue;

            // reflect origin (point)
            float const d = DotProduct(portalPlane.normal, tr.viewParms.or.origin) - portalPlane.dist;
            VectorMA(tr.viewParms.or.origin, -2.0f * d, portalPlane.normal, newParms.or.origin);

            // reflect axis (vectors)
            for(int i = 0; i < 3; i++)
            {
                float const dv = DotProduct(portalPlane.normal, tr.viewParms.or.axis[i]);
                VectorMA(tr.viewParms.or.axis[i], -2.0f * dv, portalPlane.normal, newParms.or.axis[i]);
            }
        }
        else
        {
            newParms.isMirror = qfalse;

            vec3_t offset;
            // offset = tr.viewParms.or.origin + surface.origin
            // newParms.or.origin = camera.origin + offset
            //VectorCopy(camera.origin, newParms.or.origin);
            //VectorSubtract(tr.viewParms.or.origin, surface.origin, offset);
            //VectorAdd(camera.origin, offset, newParms.or.origin);
            //AxisCopy(camera.axis, newParms.or.axis);

            R_MirrorPoint(tr.viewParms.or.origin, &surface, &camera, newParms.or.origin);
            R_MirrorVector(tr.viewParms.or.axis[0], &surface, &camera, newParms.or.axis[0]);
            R_MirrorVector(tr.viewParms.or.axis[1], &surface, &camera, newParms.or.axis[1]);
            R_MirrorVector(tr.viewParms.or.axis[2], &surface, &camera, newParms.or.axis[2]);
        }

        // Set up portal clip plane
        // newParms.portalPlane.normal = -camera.axis[0]
        // newParms.portalPlane.dist = dot(camera.origin, newParms.portalPlane.normal)
        VectorSubtract(vec3_origin, camera.axis[0], newParms.portalPlane.normal);
        newParms.portalPlane.dist = DotProduct(camera.origin, newParms.portalPlane.normal);
    }

    // OPTIMIZE: restrict the viewport on the mirrored view

    // save old viewParms so we can return to it after the mirror view
    viewParms_t oldParms = tr.viewParms;
    R_RenderView(&newParms);
    tr.viewParms = oldParms;

    return qtrue;
}
