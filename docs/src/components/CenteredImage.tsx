interface CenteredImageProps {
    src: string;
    alt: string;
    width?: number;
    borderRadius?: number;
}

export const CenteredImage = ({src, alt, width = 600, borderRadius = 8}: CenteredImageProps) => {
    return <div style={{ display: 'flex', justifyContent: 'center', paddingBottom: '1rem' }}>
        <img src={src} alt={alt} width={width} style={{ borderRadius: `${borderRadius}px` }}/>
    </div>
}